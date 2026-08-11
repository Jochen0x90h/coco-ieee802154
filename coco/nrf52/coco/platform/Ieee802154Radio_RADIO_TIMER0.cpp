#include "Ieee802154Radio_RADIO_TIMER0.hpp"
#include <coco/convert.hpp>
#include <coco/debug.hpp>
#include <coco/ieee802154.hpp>
#include <coco/platform/nvic.hpp>


namespace coco {

namespace ieee = ieee802154;


// constants
// ---------

// symbol duration: 16us
constexpr int symbolDuration = 16;

// turnaround time from receive to send and vice versa (e.g. send ack after receive)
constexpr int ackTurnaroundDuration = 12 * symbolDuration;

// maximum time to wait for an ack
constexpr int ackWaitDuration = 54 * symbolDuration;

// superframe
constexpr int baseSlotDuration = 60 * symbolDuration;
constexpr int baseSuperframeDuration = 16 * baseSlotDuration;

// inter frame spacing
constexpr int maxSifsLength = 18 - 2; // maximum short frame length without CRC
constexpr int minSifsDuration = 12 * symbolDuration;
constexpr int minLifsDuration = 40 * symbolDuration;

// backoff
constexpr int minBackoffExponent = 3;
constexpr int maxBackoffExponent = 5;
constexpr int maxBackoffCount = 3;
constexpr int unitBackoffDuration = 20 * symbolDuration;

// ack
constexpr int maxAckRetryCount = 3;


// radio state
// -----------

// radio is in Disabled state
inline bool isDisabled() {
    return NRF_RADIO->STATE == RADIO_STATE_STATE_Disabled;
}

// radio is in RxIdle state
inline bool isRxIdle() {
    return NRF_RADIO->STATE == RADIO_STATE_STATE_RxIdle;
}

// radio is in RxIdle or Rx state
inline bool isReceiveState() {
    return (NRF_RADIO->STATE | 1) == 3;
}


// interrupt
// ---------
/*
static inline void lock() {
    nvic::disable(RADIO_IRQn);
    nvic::disable(TIMER0_IRQn);
}

static inline void unlock() {
    nvic::enable(RADIO_IRQn);
    nvic::enable(TIMER0_IRQn);
}
*/


// Ieee802154Radio_RADIO_TIMER0

Ieee802154Radio_RADIO_TIMER0::Ieee802154Radio_RADIO_TIMER0(Loop_Queue &loop)
    : Ieee802154Radio(State::DISABLED)
    , loop_(loop)
{
    // init timer
    NRF_TIMER0->MODE = N(TIMER_MODE_MODE, Timer);
    NRF_TIMER0->BITMODE = N(TIMER_BITMODE_BITMODE, 32Bit);
    NRF_TIMER0->PRESCALER = 4; // 1MHz
    nvic::enable(TIMER0_IRQn);

    // init radio

    // set modulation mode
    NRF_RADIO->MODE = N(RADIO_MODE_MODE, Ieee802154_250Kbit);

    // configure crc
    NRF_RADIO->CRCCNF =
        N(RADIO_CRCCNF_SKIPADDR, Ieee802154) // CRC starts after first byte of length field
        | N(RADIO_CRCCNF_LEN, Two); // CRC-16
    NRF_RADIO->CRCPOLY = 0x11021;
    NRF_RADIO->CRCINIT = 0;

    // configure packet (PREAMBLE, ADDRESS, S0, LENGTH, S1, PAYLOAD, CRC)
    NRF_RADIO->PCNF0 =
        N(RADIO_PCNF0_PLEN, 32bitZero) // PREAMBLE is 32 bit zero
        | V(RADIO_PCNF0_S0LEN, 0) // no S0
        | N(RADIO_PCNF0_CRCINC, Include) // LENGTH includes CRC
        | V(RADIO_PCNF0_LFLEN, 8) // LENGTH is 8 bit
        | V(RADIO_PCNF0_S1LEN, 0); // no S1
    NRF_RADIO->PCNF1 =
        V(RADIO_PCNF1_MAXLEN, MAX_PAYLOAD_SIZE) // maximum length of PAYLOAD
        | N(RADIO_PCNF1_ENDIAN, Little); // little endian

    // configure clear channel assessment (CCA)
    NRF_RADIO->CCACTRL = N(RADIO_CCACTRL_CCAMODE, CarrierAndEdMode);

    // set interrupt flags and enable interrupts
    NRF_RADIO->INTENSET =
        N(RADIO_INTENSET_RXREADY, Set)
        | N(RADIO_INTENSET_EDEND, Set)
        | N(RADIO_INTENSET_CRCOK, Set)
        | N(RADIO_INTENSET_CRCERROR, Set)
        | N(RADIO_INTENSET_CCABUSY, Set)
        | N(RADIO_INTENSET_TXREADY, Set)
        | N(RADIO_INTENSET_END, Set)
        | N(RADIO_INTENSET_DISABLED, Set);
    nvic::enable(RADIO_IRQn);

    // capture time when packet was received or sent (RADIO.END -> TIMER0.CAPTURE[2])
    NRF_PPI->CHENSET = 1 << 27;
}

Ieee802154Radio_RADIO_TIMER0::~Ieee802154Radio_RADIO_TIMER0() {
}

void Ieee802154Radio_RADIO_TIMER0::close() {
    // disable timer
    NRF_TIMER0->INTENCLR = N(TIMER_INTENCLR_COMPARE0, Clear)
        | N(TIMER_INTENCLR_COMPARE1, Clear)
        | N(TIMER_INTENCLR_COMPARE3, Clear);
    NRF_TIMER0->TASKS_STOP = TRIGGER;

    {
        nvic::Guard2 guard(RADIO_IRQn, TIMER0_IRQn);

        // reset state variables
        receiverEnabled_ = false;
        endAction_ = EndAction::NOP;

        // disable radio
        NRF_RADIO->SHORTS = 0;
        NRF_RADIO->TASKS_STOP = TRIGGER;
        NRF_RADIO->TASKS_EDSTOP = TRIGGER;
        NRF_RADIO->TASKS_CCASTOP = TRIGGER;
        NRF_RADIO->TASKS_DISABLE = TRIGGER; // -> DISABLED

        // clear queues
        sendBuffers_.clear();
        for (auto &node : nodes_) {
            node.receiveBuffers_.clear();
            node.requestBuffers_.clear();
        }
    }

    // set state
    state_ = State::DISABLED;

    // set state of buffers to disabled and resume all coroutines waiting for a state change
    for (auto &node : nodes_) {
        // set state
        state_ = State::DISABLED;

        // disable buffers
        for (auto &buffer : node.buffers_) {
            buffer.setDisabled();
        }

        // set state and resume all coroutines waiting for state change
        node.notify(Device::Events::ENTER_CLOSING | Device::Events::ENTER_DISABLED);
    }

    // set state and resume all coroutines waiting for state change
    notify(Device::Events::ENTER_CLOSING | Device::Events::ENTER_DISABLED);
}

void Ieee802154Radio_RADIO_TIMER0::open(int channel) {
    // assert valid radio channel
    assert(channel >= 11 && channel <= 26);

    // stop if currently active
    if (state_ == State::READY)
        close();

    // set channel
    NRF_RADIO->FREQUENCY = V(RADIO_FREQUENCY_FREQUENCY, (channel - 10) * 5);

    // start timer
    NRF_TIMER0->TASKS_CLEAR = TRIGGER;
    NRF_TIMER0->TASKS_START = TRIGGER;

    {
        nvic::Guard2 guard(RADIO_IRQn, TIMER0_IRQn);

        // set state variables
        receiverEnabled_ = true;

        // enable radio if it is currently disabled. If not, then it gets enabled on DISABLED event in the interrupt handler
        if (isDisabled()) {
            // enable receiving mode (not the baseband decoder, packets will not be received yet)
            NRF_RADIO->TASKS_RXEN = TRIGGER; // -> RXREADY
        }
    }

    // set state
    state_ = State::READY;

    // set state of buffers to ready and resume all coroutines waiting for a state change
    for (auto &node : nodes_) {
        // set state
        node.state_ = State::READY;

        for (auto &buffer : node.buffers_) {
            buffer.setReady();
        }

        // set state and resume all coroutines waiting for state change
        node.notify(Device::Events::ENTER_OPENING | Device::Events::ENTER_READY);
    }

    // set state and resume all coroutines waiting for state change
    notify(Device::Events::ENTER_OPENING | Device::Events::ENTER_READY);
}


// Node

Ieee802154Radio_RADIO_TIMER0::Node::Node(Ieee802154Radio_RADIO_TIMER0 &device)
    : Ieee802154Radio::Node(device.state_)
    , device_(device)
{
    device.nodes_.add(*this);
}

Ieee802154Radio_RADIO_TIMER0::Node::~Node() {
}

int Ieee802154Radio_RADIO_TIMER0::Node::getBufferCount() {
    return buffers_.count();
}

coco::Buffer &Ieee802154Radio_RADIO_TIMER0::Node::getBuffer(int index) {
    return buffers_.get(index);
}

void Ieee802154Radio_RADIO_TIMER0::Node::configure(uint16_t pan, uint64_t longAddress,
    uint16_t shortAddress, FilterFlags filterFlags)
{
    longAddress_ = longAddress;
    pan_ = pan;
    shortAddress_ = shortAddress;
    filterFlags_ = filterFlags;

    // use shortAddress as random seed for backoff time
    device_.random_.reset(shortAddress | 0x12340000);
}

bool Ieee802154Radio_RADIO_TIMER0::Node::filter(uint8_t const *mac) const {
    FilterFlags flags = filterFlags_;

    // ALL: all packets pass
    if ((flags & FilterFlags::PASS_ALL) != 0)
        return true;

    // get frame control field
    auto frameControl = ieee::FrameControl(mac[0] | (mac[1] << 8));
    auto frameType = frameControl & ieee::FrameControl::TYPE_MASK;

    // reject frames with no sequence number
    if ((frameControl & ieee::FrameControl::SEQUENCE_NUMBER_SUPPRESSION) != 0)
        return false;

    // beacon packets
    if ((flags & FilterFlags::PASS_TYPE_BEACON) != 0 && frameType == ieee::FrameControl::TYPE_BEACON)
        return true;

    if ((frameControl & ieee::FrameControl::DESTINATION_ADDRESSING_FLAG) != 0) {
        // has destination address: check pan
        uint16_t pan = mac[3] | (mac[4] << 8);
        if (pan != 0xffff && pan != pan_)
            return false;

        if ((frameControl & ieee::FrameControl::DESTINATION_ADDRESSING_LONG_FLAG) == 0) {
            // short destination addressing mode
            if ((flags & FilterFlags::PASS_DEST_SHORT) != 0
                || ((flags & FilterFlags::PASS_TYPE_DATA_DEST_SHORT) != 0 && frameType == ieee::FrameControl::TYPE_DATA))
            {
                uint16_t shortAddress = mac[5] | (mac[6] << 8);
                if (shortAddress == 0xffff || shortAddress == shortAddress_)
                    return true;
            }
        } else {
            // long destination addressing mode
            if ((flags & FilterFlags::PASS_DEST_LONG) != 0) {
                uint32_t longAddressLo = mac[5] | (mac[6] << 8) | (mac[7] << 16) | (mac[8] << 24);
                uint32_t longAddressHi = mac[9] | (mac[10] << 8) | (mac[11] << 16) | (mac[12] << 24);
                if (longAddressLo == uint32_t(longAddress_) && longAddressHi == uint32_t(longAddress_ >> 32))
                    return true;
            }
        }
    }

    return false;
}

bool Ieee802154Radio_RADIO_TIMER0::Node::request(uint16_t panId, const uint8_t *destinationAddress, int addressLength) {
    // method is called only from interrupt handler, therefore no locking required
    /*bool result = requestBuffers_.remove([this, panId, destinationAddress, addressLength](Buffer &buffer) {
        auto mac = buffer.data_;

        // check pan id
        if (panId != (mac[3] | (mac[4] << 8)))
            return false;

        // frame control
        auto frameControl = ieee::FrameControl(mac[0] | (mac[1] << 8))
            & (ieee::FrameControl::SEQUENCE_NUMBER_SUPPRESSION | ieee::FrameControl::DESTINATION_ADDRESSING_MASK);

        // check destination addressing
        if (addressLength == 2) {
            if (frameControl != ieee::FrameControl::DESTINATION_ADDRESSING_SHORT)
                return false;
        } else {
            if (frameControl != ieee::FrameControl::DESTINATION_ADDRESSING_LONG)
                return false;
        }

        // check destination address
        if (!std::equal(destinationAddress, destinationAddress + addressLength, mac + 5))
            return false;

        // move buffer to sendBuffers
        device_.sendBuffers_.push(buffer);
        return true;
    });*/
    auto buffer = requestBuffers_.findAndRemove(
        [this, panId, destinationAddress, addressLength](Buffer &buffer, int) {
            auto mac = buffer.data_;

            // check pan id
            if (panId != (mac[3] | (mac[4] << 8)))
                return false;

            // frame control
            auto frameControl = ieee::FrameControl(mac[0] | (mac[1] << 8))
                & (ieee::FrameControl::SEQUENCE_NUMBER_SUPPRESSION | ieee::FrameControl::DESTINATION_ADDRESSING_MASK);

            // check destination addressing
            if (addressLength == 2) {
                if (frameControl != ieee::FrameControl::DESTINATION_ADDRESSING_SHORT)
                    return false;
            } else {
                if (frameControl != ieee::FrameControl::DESTINATION_ADDRESSING_LONG)
                    return false;
            }

            // check destination address
            if (!std::equal(destinationAddress, destinationAddress + addressLength, mac + 5))
                return false;

            return true;
        });
    if (buffer != nullptr) {
        // move buffer to sendBuffers
        device_.sendBuffers_.push(*buffer);
        return true;
    }
    return false;
}


// Buffer

Ieee802154Radio_RADIO_TIMER0::Buffer::Buffer(Node &node)
    : coco::Buffer(buffer_, HEADER_SIZE, MAX_PAYLOAD_SIZE, node.state_)
    , node_(node)
{
    node.buffers_.add(*this);
}

Ieee802154Radio_RADIO_TIMER0::Buffer::~Buffer() {
}

bool Ieee802154Radio_RADIO_TIMER0::Buffer::start() {
    if (state_ != State::READY) {
        assert(false);
        setError(std::errc::resource_unavailable_try_again);
        return false;
    }
    if ((op_ & Op::READ_WRITE) == 0 || size_ == 0) {
        setSuccess();
        return false;
    }
    //debug::out << ((op_ & Op::WRITE) != 0 ? "write" : "read") << ' ' << dec(size_) << '\n';
    auto &node = node_;
    auto &device = node.device_;

    {
        nvic::Guard2 guard(RADIO_IRQn, TIMER0_IRQn);

        if ((op_ & Op::WRITE) == 0) {
            // receive
            node.receiveBuffers_.push(*this);
            mode_ = Mode::RECEIVE;
        } else {
            // send
            auto sendFlags = SendFlags(header_[0]);
            if (sendFlags == SendFlags::AWAIT_DATA_REQUEST) {
                // need to wait for a data request from the receiver of this packet
                node.requestBuffers_.push(*this);
                mode_ = Mode::REQUEST;
            } else {
                // packet can be sent as soon as possible
                device.sendBuffers_.push(*this);
                mode_ = Mode::SEND;

                // check if we can start sending now
                if (device.sendState_ == SendState::IDLE) {
                    device.prepareForSend(*this);
                }
            }
        }
    }

    // set state
    setBusy();

    return true;
}

bool Ieee802154Radio_RADIO_TIMER0::Buffer::cancel() {
    if (state_ != State::BUSY)
        return false;
    auto &node = node_;
    auto &device = node.device_;

    bool success = true;
    {
        nvic::Guard2 guard(RADIO_IRQn, TIMER0_IRQn);
        switch (mode_) {
        case Mode::RECEIVE:
            // remove from pending receive transfers
            node.receiveBuffers_.remove(*this);
            break;
        case Mode::REQUEST:
            // remove from pending request buffers
            node.requestBuffers_.remove(*this);
            break;
        case Mode::SEND:
            // remove from pending send transfers if not yet started, otherwise complete normally
            success = device.sendBuffers_.removeExceptFirst(*this);
            break;
        }
    }
    if (success) {
        setError(std::errc::operation_canceled);
        setReady();
    }

    return true;
}

void Ieee802154Radio_RADIO_TIMER0::Buffer::onCompletion() {
    setReady();
}


// Ieee802154Radio_RADIO_TIMER0 protected:

void Ieee802154Radio_RADIO_TIMER0::startReceive() {
    endAction_ = EndAction::RECEIVE;

    NRF_RADIO->PACKETPTR = uintptr_t(receivePacket_);
    NRF_RADIO->TASKS_START = TRIGGER;

    // -> CRCOK or CRCERROR, END
}

void Ieee802154Radio_RADIO_TIMER0::selectForSend() {
    // select next send buffer
    if (!sendBuffers_.empty())
        prepareForSend(sendBuffers_.front());
    //Buffer *buffer = sendBuffers_.frontOrNull();
    //if (buffer != nullptr)
    //    prepareForSend(*buffer);
}

void Ieee802154Radio_RADIO_TIMER0::prepareForSend(Buffer &buffer) {
    // check if the ACK request flag is set
    auto mac = buffer.data_;// + buffer.p.headerSize;
    bool requestAck = (ieee::FrameControl(mac[0]) & ieee::FrameControl::ACKNOWLEDGE_REQUEST) != 0;

    // check if the node that owns the buffer is configured for handling ACK
    auto flags = buffer.node_.filterFlags_;
    bool handleAck = (flags & FilterFlags::HANDLE_ACK) != 0;

    // either wait until packet is sent or until ack was received
    sendState_ = (requestAck && handleAck) ? SendState::AWAIT_SENT_ACK : SendState::AWAIT_SENT;

    // reset ack retry count
    ackRetryCount_ = 1;

    // start backoff for csma-ca
#ifndef NO_CSMA_CA
    startBackoff();
#else
    NRF_RADIO->TASKS_STOP = TRIGGER;
    NRF_RADIO->TASKS_TXEN = TRIGGER; // -> TXREADY
#endif
}

void Ieee802154Radio_RADIO_TIMER0::startBackoff() {
    // initialize backoff parameters
    backoffExponent_ = minBackoffExponent;
    backoffCount_ = 0;

    // first backoff for csma-ca
    backoff();
}

void Ieee802154Radio_RADIO_TIMER0::backoff() {
    // fail when maximum backoff count is reached
    if (backoffCount_ >= maxBackoffCount) {
//debug::set(debug::CYAN);
        finishSend(std::errc::resource_unavailable_try_again);

        // check if more to send
        selectForSend();

        return;
    }

    // throw the dice to determine backoff period
    int backoff = (random_.draw() & ~(0xffffffff << backoffExponent_)) + 1;
    int backoffDuration = backoff * unitBackoffDuration;

    // update backoff parameters
    backoffExponent_ = std::min(backoffExponent_ + 1, maxBackoffExponent);
    ++backoffCount_;

    // set timer for backoff duration
    NRF_TIMER0->TASKS_CAPTURE[3] = TRIGGER;
    NRF_TIMER0->CC[3] = NRF_TIMER0->CC[3] + backoffDuration;
    NRF_TIMER0->EVENTS_COMPARE[3] = 0;
    NRF_TIMER0->INTENSET = N(TIMER_INTENSET_COMPARE3, Set);

    // -> COMPARE[3]
}

void Ieee802154Radio_RADIO_TIMER0::startClearChannelAssessment() {
    //debug::out << "start CCA\n";

    // shortcut: stop receiving and enable sender if channel is clear
    NRF_RADIO->SHORTS = N(RADIO_SHORTS_CCAIDLE_STOP, Enabled)
        | N(RADIO_SHORTS_CCAIDLE_TXEN, Enabled);

    // start clear channel assessment
    NRF_RADIO->TASKS_CCASTART = TRIGGER;

    // -> CCABUSY or CCAIDLE -> TXEN -> TXREADY
}

void Ieee802154Radio_RADIO_TIMER0::startSend() {
    // handle sent packet on END event
    endAction_ = EndAction::FINISH_SEND;

    auto &buffer =  sendBuffers_.front();
    auto packet = buffer.data_ - 1; // one byte before paylaod for length (overwrites last byte of header)
    int length = buffer.size_ + 2; // set packet length including space for CRC

    // set packet length before payload
    packet[0] = length;

    // determine inter frame spacing duration
    ifsDuration_ = length <= maxSifsLength ? minSifsDuration : minLifsDuration;

    //debug::out << "startSend " << dec(length) << " ifs " << dec(ifsDuration_) << '\n';

    // start send operation
    NRF_RADIO->PACKETPTR = uintptr_t(packet);
    NRF_RADIO->TASKS_START = TRIGGER;

    // -> END
}

void Ieee802154Radio_RADIO_TIMER0::startSendAck() {
    // don't do anything on END event
    endAction_ = EndAction::NOP;

    NRF_RADIO->PACKETPTR = intptr_t(ackPacket_);
    NRF_RADIO->TASKS_START = TRIGGER;

    // -> END
}

void Ieee802154Radio_RADIO_TIMER0::finishSend(std::errc error) {
    //debug::out << "finishSend " << dec(error) << '\n';
    sendBuffers_.pop(
        [this, error](Buffer &buffer) {
            if (error != std::errc{}) {
                buffer.setError(error);
            } else {
                buffer.setSuccess();
            }
            loop_.push(buffer);
        });

    // sender is idle again
    sendState_ = SendState::IDLE;
}

void Ieee802154Radio_RADIO_TIMER0::RADIO_IRQHandler() {
    // check if ready to receive (RxIdle state)
    if (NRF_RADIO->EVENTS_RXREADY) {
        NRF_RADIO->EVENTS_RXREADY = 0;

        // start receive
        if (receiverEnabled_)
            startReceive(); // -> END

        // check if something to send (this is executed after start or send)
        if (sendState_ == SendState::IDLE) {
            selectForSend();
        }
    }

    // check if end of energy detection
    if (NRF_RADIO->EVENTS_EDEND) {
        NRF_RADIO->EVENTS_EDEND = 0;

        // signal end of energy detection to handle()
        NRF_EGU0->TASKS_TRIGGER[0] = TRIGGER;
    }

    // check if clear channel assessment failed
    if (NRF_RADIO->EVENTS_CCABUSY) {
        NRF_RADIO->EVENTS_CCABUSY = 0;

        // backoff and try again
        backoff();
    }

    // check if sender ready (TxIdle state)
    if (NRF_RADIO->EVENTS_TXREADY) {
        NRF_RADIO->EVENTS_TXREADY = 0;
        // shortcut: disable sender on END event
        NRF_RADIO->SHORTS = N(RADIO_SHORTS_END_DISABLE, Enabled);

        // check if clear channel assessment was running
#ifndef NO_CSMA_CA
        if (endAction_ == EndAction::SEND_ACK) {
            // send ACK of last received packet overrides a normal send operation
            if (sendState_ != SendState::IDLE) {
                // backoff the normal send operation and try again
                backoff();
            }

            // send carrier until startSendAck() happens in timer interrupt after ACK turnaround time
        } else {
            // start send operation assuming that CCA was successful (CCAIDLE)
            startSend();
        }
#else
        startSend();
#endif
    }

    // check if packet was received with no crc error
    if (NRF_RADIO->EVENTS_CRCOK) {
        NRF_RADIO->EVENTS_CRCOK = 0;

        // get size of payload data (subtract 2 for crc)
        int size = receivePacket_[0] - 2;
        const uint8_t *mac = receivePacket_ + 1; // skip length byte

        // determine inter frame spacing duration
        ifsDuration_ = size <= maxSifsLength ? minSifsDuration : minLifsDuration;

        // get frame control
        auto frameControl = ieee::FrameControl(mac[0] | (mac[1] << 8));

        // check if a previously sent packet awaits an ACK
        if (sendState_ == SendState::AWAIT_ACK) {
            // check if this is the ACK packet that we are waiting for
            auto frameType = frameControl & ieee::FrameControl::TYPE_MASK;
            if (frameType == ieee::FrameControl::TYPE_ACK) {
                // check if mac counter of sent packet matches mac counter of ACK packet
                auto &sentBuffer = sendBuffers_.front();
                if (mac[2] == sentBuffer.data_[2]) {
                    // yes: disable interrupts of ACK wait and backoff
                    NRF_TIMER0->INTENCLR =
                        N(TIMER_INTENCLR_COMPARE1, Clear)
                        | N(TIMER_INTENCLR_COMPARE3, Clear);

                    // set state of sent packet to success, send state becomes idle
    //debug::set(debug::YELLOW);
                    finishSend();

                    // check if more to send
                    selectForSend();
                }
            }
        }

        // capture timestamp of received packet (CC[1] is currently unused)
        //if (RECEIVE_HEADER_SIZE >= 5) {
            NRF_TIMER0->TASKS_CAPTURE[1] = TRIGGER;
        //}

        // check if a node is interested in this packet and wants to handle ack
        bool ack = false;
        ackPacket_[1] = uint8_t(ieee::FrameControl::TYPE_ACK);
        //bool listening = false;
        for (auto &node : nodes_) {
            // check if the node wants to receive the packet
            if (node.filterFlags_ != FilterFlags::NONE && node.filter(mac)) {
                bool passThis = true;

                // check if this node handles ack and the packet requests an ack
                if ((node.filterFlags_ & FilterFlags::HANDLE_ACK) != 0
                    && (frameControl & ieee::FrameControl::ACKNOWLEDGE_REQUEST) != 0)
                {
                    // we need to send an ack
                    ack = true;

                    // check if this is a data request packet
                    if ((frameControl & (ieee::FrameControl::TYPE_MASK
                            | ieee::FrameControl::SECURITY
                            | ieee::FrameControl::PAN_ID_COMPRESSION
                            | ieee::FrameControl::SEQUENCE_NUMBER_SUPPRESSION
                            | ieee::FrameControl::DESTINATION_ADDRESSING_FLAG
                            | ieee::FrameControl::SOURCE_ADDRESSING_FLAG))
                        == (ieee::FrameControl::TYPE_COMMAND
                            | ieee::FrameControl::PAN_ID_COMPRESSION
                            | ieee::FrameControl::DESTINATION_ADDRESSING_FLAG
                            | ieee::FrameControl::SOURCE_ADDRESSING_FLAG))
                    {
                        // frameControl and macCounter
                        int i = 2 + 1;

                        // pan id
                        uint16_t panId = mac[i] | (mac[i + 1] << 8);
                        i += 2;

                        // skip destination address
                        if ((frameControl & ieee::FrameControl::DESTINATION_ADDRESSING_LONG_FLAG) != 0)
                            i += 8;
                        else
                            i += 2;

                        // source address
                        const uint8_t *sourceAddress = mac + i;
                        int len = (frameControl & ieee::FrameControl::SOURCE_ADDRESSING_LONG_FLAG) == 0 ? 2 : 8;

                        // check if it is a data request command
                        if (mac[i + len] == uint8_t(ieee::Command::DATA_REQUEST)) {
                            // check if there is a packet pending for this source and if yes, move to sendBuffers
                            if (node.request(panId, sourceAddress, len)) {
                                // set frame pending flag in ACK packet
                                ackPacket_[1] = uint8_t(ieee::FrameControl::TYPE_ACK | ieee::FrameControl::FRAME_PENDING);
                            }

                            // don't pass data request packet to node
                            passThis = false;
                        }
                    }
                }
                if (passThis) {
                    node.receiveBuffers_.pop(
                        [this, mac, size](Buffer &buffer) {
                            // set header
                            ReceiveHeader &header = *reinterpret_cast<ReceiveHeader *>(buffer.header_);

                            // timestamp
                            header.timestamp = NRF_TIMER0->CC[1];

                            // link quality indicator (LQI)
                            header.lqi = mac[size];

                            // copy payload to buffer
                            buffer.size_ = size;
                            std::copy(mac, mac + size, buffer.data_);

                            // pass buffer to event loop so that the main application gets notified
                            loop_.push(buffer);
                            //return true;
                        });
                }
            }
            //listening |= !node.receiveBuffers.empty();
        }

        // check if we need to send an ACK
        if (ack) {
            // note: clear channel assessment for a normal send operation may be in progress at this point. This means
            // that if the sender gets enabled (TXREADY), ACK has to override the normal send operation

            // copy frame sequence number to ACK packet
            ackPacket_[3] = receivePacket_[3];

            // send ACK on END event
            endAction_ = EndAction::SEND_ACK;
        }
    }

    // check if packet was received with crc error
    if (NRF_RADIO->EVENTS_CRCERROR) {
        NRF_RADIO->EVENTS_CRCERROR = 0;

        // worst case assumption as we don't know how long the packet was
        ifsDuration_ = minLifsDuration;
    }

    // check if transfer has ended (receive or send)
    if (NRF_RADIO->EVENTS_END) {
        NRF_RADIO->EVENTS_END = 0;

        // capture time when last packet was sent or received (done automatically by PPI channel 27)
        //NRF_TIMER0->TASKS_CAPTURE[2] = TRIGGER;

        // disable after send (done automatically by SHORTS)
        //if (sendState != SendState::IDLE)
        //	NRF_RADIO->TASKS_DISABLE = TRIGGER; // -> DISABLED

        // decide what to do next
        switch (endAction_) {
        case EndAction::NOP:
            // do nothing
            break;
        case EndAction::RECEIVE:
            if (receiverEnabled_) {
                // continue receiving
                startReceive();

                // -> CRCOK or CRCERROR, END
            }
            break;
        case EndAction::SEND_ACK:
            // enable sender for sending ack
            NRF_RADIO->TASKS_TXEN = TRIGGER; // -> TXREADY

            // set timer for ack turnaround duration
            NRF_TIMER0->CC[0] = NRF_TIMER0->CC[2] + ackTurnaroundDuration;
            NRF_TIMER0->EVENTS_COMPARE[0] = 0;
            NRF_TIMER0->INTENSET = N(TIMER_INTENSET_COMPARE0, Set); // -> COMPARE[0]

            // now wait for timeout
            break;
        case EndAction::FINISH_SEND:
            // sent a packet, is either finished or we need to wait for ACK
            {
                // check if we have to wait for an ACK
                if (sendState_ == SendState::AWAIT_SENT_ACK) {
                    // yes: set timer for ACK wait duration
                    NRF_TIMER0->CC[1] = NRF_TIMER0->CC[2] + ackWaitDuration;
                    NRF_TIMER0->EVENTS_COMPARE[1] = 0;
                    NRF_TIMER0->INTENSET = N(TIMER_INTENSET_COMPARE1, Set); // -> COMPARE[1]

                    // now we either receive an ACK packet or timeout
                    sendState_ = SendState::AWAIT_ACK;
                    //receiverEnabled_ = true;
                } else {
                    // no: set state of sent packet to success, send state becomes idle
    //debug::set(debug::BLUE);
                    finishSend();
                }
            }
            break;
        }
    }

    // check if sender has been disabled (either by stop() or after send)
    if (NRF_RADIO->EVENTS_DISABLED) {
        NRF_RADIO->EVENTS_DISABLED = 0;

        // clear shortcut END -> DISABLE
        NRF_RADIO->SHORTS = 0;

        // enable receiver again if radio is active
        if (receiverEnabled_) {
        //if (st.state == State::READY) {
            NRF_RADIO->TASKS_RXEN = TRIGGER; // -> RXREADY
        }
    }
}

void Ieee802154Radio_RADIO_TIMER0::TIMER0_IRQHandler() {
    // read interrupt enable flags
    uint32_t INTEN = NRF_TIMER0->INTENSET;

    // check if ack turnaround time (send ACK after receive) elapsed
    if (TEST(INTEN, TIMER_INTENSET_COMPARE0, Enabled) && NRF_TIMER0->EVENTS_COMPARE[0]) {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;

        // disable timer interrupt 0
        NRF_TIMER0->INTENCLR = N(TIMER_INTENCLR_COMPARE0, Clear);

        // send ack
        startSendAck();
    }

    // check if ACK wait duration (receive ACK after send) has elapsed
    if (TEST(INTEN, TIMER_INTENSET_COMPARE1, Enabled) && NRF_TIMER0->EVENTS_COMPARE[1]) {
        NRF_TIMER0->EVENTS_COMPARE[1] = 0;

        // disable timer interrupt 1
        NRF_TIMER0->INTENCLR = N(TIMER_INTENCLR_COMPARE1, Clear);

        if (ackRetryCount_ < maxAckRetryCount) {
            // ack was not received: retry
            sendState_ = SendState::AWAIT_SENT_ACK;
            ++ackRetryCount_;
            startBackoff();
        } else {
//debug::set(debug::RED);
            // sent packet was not acknowledged: set result to failed, send state becomes idle
            finishSend(std::errc::timed_out);

            // todo: disable receiver (and set receiverenabled_ = false) after a timeout when no node listens

            // check if more to send
            selectForSend();
        }
    }

    // check if backoff period elapsed
    if (TEST(INTEN, TIMER_INTENSET_COMPARE3, Enabled) && NRF_TIMER0->EVENTS_COMPARE[3]) {
        NRF_TIMER0->EVENTS_COMPARE[3] = 0;

        // disable timer interrupt 3
        NRF_TIMER0->INTENCLR = N(TIMER_INTENCLR_COMPARE3, Clear);

        // check if in receive state
        bool s = isReceiveState();

        // get time since last packet was received or sent
        NRF_TIMER0->TASKS_CAPTURE[3] = TRIGGER;
        uint32_t t = NRF_TIMER0->CC[3] - NRF_TIMER0->CC[2];

        // check if the radio is in the right state and inter frame spacing is respected
        if (s && t >= ifsDuration_) {
            // start clear channel assessment
            startClearChannelAssessment();
        } else {
            // backoff and try again
            backoff();
        }
    }
}

} // namespace coco
