#include "Ieee802154Radio_RADIO_TIMER0_EGU0.hpp"
//#include <coco/debug.hpp>
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

static inline void lock() {
	nvic::disable(RADIO_IRQn);
	nvic::disable(TIMER0_IRQn);
}

static inline void unlock() {
	nvic::enable(RADIO_IRQn);
	nvic::enable(TIMER0_IRQn);
}



// Ieee802154Radio_RADIO_TIMER0_EGU0

Ieee802154Radio_RADIO_TIMER0_EGU0::Ieee802154Radio_RADIO_TIMER0_EGU0(Loop_RTC0 &loop) {
	Ieee802154Radio_RADIO_TIMER0_EGU0::instance = this;

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
		V(RADIO_PCNF1_MAXLEN, MAX_PAYLOAD_LENGTH) // maximum length of PAYLOAD
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

	// capture time when packet was received or sent (RADIO:END -> TIMER0:CAPTURE[2])
	NRF_PPI->CHENSET = 1 << 27;

	// init event generator
	NRF_EGU0->INTENSET =
		N(EGU_INTENSET_TRIGGERED0, Set)
		| N(EGU_INTENSET_TRIGGERED1, Set)
		| N(EGU_INTENSET_TRIGGERED2, Set);

	loop.handlers.add(*this);
}

Ieee802154Radio_RADIO_TIMER0_EGU0::~Ieee802154Radio_RADIO_TIMER0_EGU0() {
}

void Ieee802154Radio_RADIO_TIMER0_EGU0::start(int channel) {
	assert(channel >= 11 && channel <= 26);

	// stop if currently active
	//if (this->active)
	if (this->stat == State::READY)
		stop();

	// set channel
	NRF_RADIO->FREQUENCY = V(RADIO_FREQUENCY_FREQUENCY, (channel - 10) * 5);

	// start timer
	NRF_TIMER0->TASKS_CLEAR = TRIGGER;
	NRF_TIMER0->TASKS_START = TRIGGER;

	lock();

	// enable radio if it is currently disabled. If not, then it gets enabled on DISABLED event in the interrupt handler
	if (isDisabled()) {
		// enable receiving mode (not the baseband decoder, packets will not be received yet)
		NRF_RADIO->TASKS_RXEN = TRIGGER; // -> RXREADY
	}

	// set state
	this->stat = State::READY;

	unlock();

	// set state of buffers to ready
	for (auto &node : this->nodes) {
		for (auto &buffer : node.buffers) {
			buffer.setReady(0);
		}
	}

	// resume all coroutines waiting for a state change
	this->stateTasks.doAll();
}

void Ieee802154Radio_RADIO_TIMER0_EGU0::stop() {
	// disable timer
	NRF_TIMER0->INTENCLR = N(TIMER_INTENCLR_COMPARE0, Clear)
		| N(TIMER_INTENCLR_COMPARE1, Clear)
		| N(TIMER_INTENCLR_COMPARE3, Clear);
	NRF_TIMER0->TASKS_STOP = TRIGGER;

	lock();

	// reset state variables
	this->stat = State::DISABLED;
	this->receiverEnabled = false;
	this->endAction = EndAction::NOP;

	// disable radio
	NRF_RADIO->SHORTS = 0;
	NRF_RADIO->TASKS_STOP = TRIGGER;
	NRF_RADIO->TASKS_EDSTOP = TRIGGER;
	NRF_RADIO->TASKS_CCASTOP = TRIGGER;
	NRF_RADIO->TASKS_DISABLE = TRIGGER; // -> DISABLED

	// set state of buffers to disabled
	for (auto &node : this->nodes) {
		for (auto &buffer : node.buffers) {
			// cancel (remove from receiveBuffers, requestBuffers or sendBuffers)
			buffer.remove2();
			unlock();
			buffer.xferred = 0;
			buffer.setDisabled();
			lock();
		}
	}

	unlock();

	// resume all coroutines waiting for a state change
	this->stateTasks.doAll();
}


// Ieee802154_RADIO_TIMER0_EGU0::Node

Ieee802154Radio_RADIO_TIMER0_EGU0::Node::Node(Ieee802154Radio_RADIO_TIMER0_EGU0 &radio)
	: radio(radio)
{
	radio.nodes.add(*this);
}

Ieee802154Radio_RADIO_TIMER0_EGU0::Node::~Node() {
}

Device::State Ieee802154Radio_RADIO_TIMER0_EGU0::Node::state() {
	return this->radio.stat;
}

Awaitable<> Ieee802154Radio_RADIO_TIMER0_EGU0::Node::stateChange(int waitFlags) {
	if ((waitFlags & (1 << int(State::READY))) == 0)
		return {};
	return {this->radio.stateTasks};
}

int Ieee802154Radio_RADIO_TIMER0_EGU0::Node::getBufferCount() {
	return this->buffers.count();
}

coco::Buffer &Ieee802154Radio_RADIO_TIMER0_EGU0::Node::getBuffer(int index) {
	return this->buffers.get(index);
}

void Ieee802154Radio_RADIO_TIMER0_EGU0::Node::configure(uint16_t pan, uint64_t longAddress,
	uint16_t shortAddress, FilterFlags filterFlags)
{
	this->longAddress = longAddress;
	this->pan = pan;
	this->shortAddress = shortAddress;
	this->filterFlags = filterFlags;

	// use shortAddress as additional random seed
	this->radio.random.reset((this->radio.random.draw() + shortAddress) | 0x10000);
}

bool Ieee802154Radio_RADIO_TIMER0_EGU0::Node::filter(uint8_t const *data) const {
	uint8_t const *mac = data + 1;
	FilterFlags flags = this->filterFlags;

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
		if (pan != 0xffff && pan != this->pan)
			return false;

		if ((frameControl & ieee::FrameControl::DESTINATION_ADDRESSING_LONG_FLAG) == 0) {
			// short destination addressing mode
			if ((flags & FilterFlags::PASS_DEST_SHORT) != 0
				|| ((flags & FilterFlags::PASS_TYPE_DATA_DEST_SHORT) != 0 && frameType == ieee::FrameControl::TYPE_DATA))
			{
				uint16_t shortAddress = mac[5] | (mac[6] << 8);
				if (shortAddress == 0xffff || shortAddress == this->shortAddress)
					return true;
			}
		} else {
			// long destination addressing mode
			if ((flags & FilterFlags::PASS_DEST_LONG) != 0) {
				uint32_t longAddressLo = mac[5] | (mac[6] << 8) | (mac[7] << 16) | (mac[8] << 24);
				uint32_t longAddressHi = mac[9] | (mac[10] << 8) | (mac[11] << 16) | (mac[12] << 24);
				if (longAddressLo == uint32_t(this->longAddress) && longAddressHi == uint32_t(this->longAddress >> 32))
					return true;
			}
		}
	}

	return false;
}

bool Ieee802154Radio_RADIO_TIMER0_EGU0::Node::request(uint16_t panId, const uint8_t *destinationAddress, int addressLength) {
	for (auto &buffer : this->requestBuffers) {
		// check pan id
		if (panId != (buffer.packet[4] | (buffer.packet[5] << 8)))
			continue;

		// frame control
		auto frameControl = ieee::FrameControl(buffer.packet[1] | (buffer.packet[2] << 8))
			& (ieee::FrameControl::SEQUENCE_NUMBER_SUPPRESSION | ieee::FrameControl::DESTINATION_ADDRESSING_MASK);

		// check destination addressing
		if (addressLength == 2) {
			if (frameControl != ieee::FrameControl::DESTINATION_ADDRESSING_SHORT)
				continue;
		} else {
			if (frameControl != ieee::FrameControl::DESTINATION_ADDRESSING_LONG)
				continue;
		}

		// check destination address
		if (std::equal(destinationAddress, destinationAddress + addressLength, buffer.packet + 6)) {
		//if (array::equals(addressLength, destinationAddress, buffer.packet + 6)) {
			// clear AWAIT_DATA_REQUEST flag in byte behind the packet
			//int length = buffer.packet[0] - 2;
			//buffer.packet[1 + length] &= uint8_t(~SendFlags::AWAIT_DATA_REQUEST);
			buffer.remove2();
			this->radio.sendBuffers.add(buffer);
			return true;
		}
	}
	return false;
}


// Ieee802154_RADIO_TIMER0_EGU0::Buffer

Ieee802154Radio_RADIO_TIMER0_EGU0::Buffer::Buffer(Node &node)
	: BufferImpl(packet + 1, PACKET_LENGTH, node.radio.stat), node(node)
{
	node.buffers.add(*this);
}

Ieee802154Radio_RADIO_TIMER0_EGU0::Buffer::~Buffer() {
}

void Ieee802154Radio_RADIO_TIMER0_EGU0::Buffer::cancel() {
	if (this->stat != State::BUSY)
		return;

	if (this != this->node.radio.sendBuffer) {
		// cancel can take effect immediately
		lock();
		this->remove2();
		unlock();
		setReady(0);
	} else {
		// sending of this buffer is in progress: cancel has no effect
	}
}

bool Ieee802154Radio_RADIO_TIMER0_EGU0::Buffer::startInternal(int size, Op op) {
	if (this->stat != State::READY) {
		assert(this->stat != State::BUSY);
		return false;
	}

	// check if READ or WRITE flag is set
	assert((op & Op::READ_WRITE) != 0);

	lock();
	if ((op & Op::WRITE) == 0) {
		// receive
		this->node.receiveBuffers.add(*this);

		this->node.radio.receiverEnabled = true;

		// start receiver (base band decoder) if radio is in RxIdle state, otherwise do it on RXREADY
		if (isRxIdle()) {
			// todo: maybe there is a race condition if the radio automatically switches to TXREADY after CCA here. maybe wait until CCA has ended
			this->node.radio.startReceive(); // -> END
		}
	} else {
		// send
		this->xferred = size;
		size -= SEND_EXTRA_LENGTH;
		this->packet[0] = size + 2; // space for CRC
		auto sendFlags = SendFlags(this->packet[1 + size]);
		if (sendFlags == SendFlags::AWAIT_DATA_REQUEST) {
			// need to wait for a data request from the receiver of this packet
			this->node.requestBuffers.add(*this);
		} else {
			// packet can be sent as soon as possible
			this->node.radio.sendBuffers.add(*this);

			// check if we can start sending now
			if (this->node.radio.sendState == SendState::IDLE) {
				this->node.radio.prepareForSend(*this);
			}
		}
	}
	this->finished = false;
	unlock();

	// set state
	setBusy();

	return true;
}


// Ieee802154Radio_RADIO_TIMER0_EGU0 protected:

void Ieee802154Radio_RADIO_TIMER0_EGU0::handle() {
	if (nvic::pending(SWI0_EGU0_IRQn)) {
		// check energy detection
		if (NRF_EGU0->EVENTS_TRIGGERED[0]) {
			NRF_EGU0->EVENTS_TRIGGERED[0] = 0;
			//Radio::onEdReady(NRF_RADIO->EDSAMPLE);
		}

		// check if a send operation has finished
		if (NRF_EGU0->EVENTS_TRIGGERED[2]) {
			NRF_EGU0->EVENTS_TRIGGERED[2] = 0;

			// handle all finished send buffers
			lock();
			while (!this->sendBuffers.empty()) {
				auto &buffer = *this->sendBuffers.begin();
				if (!buffer.finished)
					break;
				buffer.remove2();
				unlock();
				buffer.setReady();
				lock();
			}
			unlock();
		}

		// check if receive operation has finished
		if (NRF_EGU0->EVENTS_TRIGGERED[1]) {
			NRF_EGU0->EVENTS_TRIGGERED[1] = 0;

			// handle all finished receive buffers
			lock();
			for (auto &channel : this->nodes) {
				while (!channel.receiveBuffers.empty()) {
					auto &buffer = *channel.receiveBuffers.begin();
					if (!buffer.finished)
						break;
					buffer.remove2();
					unlock();
					buffer.setReady();
					lock();
				}
			}
			unlock();
		}

		// clear pending interrupt flag at NVIC
		nvic::clear(SWI0_EGU0_IRQn);
	}
}

void Ieee802154Radio_RADIO_TIMER0_EGU0::startReceive() {
	this->endAction = EndAction::RECEIVE;

	NRF_RADIO->PACKETPTR = intptr_t(this->receivePacket);
	NRF_RADIO->TASKS_START = TRIGGER; // -> END
}

void Ieee802154Radio_RADIO_TIMER0_EGU0::selectForSend() {
	// select next send buffer
	for (auto &buffer : this->sendBuffers) {
		if (!buffer.finished) {
			prepareForSend(buffer);
			break;
		}
	}
}

void Ieee802154Radio_RADIO_TIMER0_EGU0::prepareForSend(Buffer &buffer) {
	auto flags = buffer.node.filterFlags;

	// check if the packet requests an ACK
	bool requestAck = (ieee::FrameControl(buffer.packet[1]) & ieee::FrameControl::ACKNOWLEDGE_REQUEST) != 0;

	// check if the context is configured for handling ACK
	bool handleAck = (flags & FilterFlags::HANDLE_ACK) != 0;

	// set current send buffer
	this->sendBuffer = &buffer;

	// either wait until packet is sent or until ack was received
	this->sendState = (requestAck && handleAck) ? SendState::AWAIT_SENT_ACK : SendState::AWAIT_SENT;

	// reset ack retry count
	this->ackRetryCount = 1;

	// start backoff for csma-ca
#ifndef NO_CSMA_CA
	startBackoff();
#else
	NRF_RADIO->TASKS_STOP = TRIGGER;
	NRF_RADIO->TASKS_TXEN = TRIGGER; // -> TXREADY
#endif
}

void Ieee802154Radio_RADIO_TIMER0_EGU0::startBackoff() {
	// initialize backoff parameters
	this->backoffExponent = minBackoffExponent;
	this->backoffCount = 0;

	// first backoff for csma-ca
	backoff();
}

void Ieee802154Radio_RADIO_TIMER0_EGU0::backoff() {
	// fail when maximum backoff count is reached
	if (this->backoffCount >= maxBackoffCount) {
		this->sendBuffer->xferred = 0; // indicate failure
//debug::set(debug::CYAN);
		finishSend();

		// check if more to send
		selectForSend();

		return;
	}

	// throw the dice to determine backoff period
	int backoff = (this->random.draw() & ~(0xffffffff << this->backoffExponent)) + 1;
	int backoffDuration = backoff * unitBackoffDuration;

	// update backoff parameters
	this->backoffExponent = std::min(this->backoffExponent + 1, maxBackoffExponent);
	++this->backoffCount;

	// set timer for backoff duration
	NRF_TIMER0->TASKS_CAPTURE[3] = TRIGGER;
	NRF_TIMER0->CC[3] = NRF_TIMER0->CC[3] + backoffDuration;
	NRF_TIMER0->EVENTS_COMPARE[3] = 0;
	NRF_TIMER0->INTENSET = N(TIMER_INTENSET_COMPARE3, Set); // -> COMPARE[3]
}

void Ieee802154Radio_RADIO_TIMER0_EGU0::startClearChannelAssessment() {
	// shortcut: stop receiving and enable sender if channel is clear
	NRF_RADIO->SHORTS = N(RADIO_SHORTS_CCAIDLE_STOP, Enabled)
		| N(RADIO_SHORTS_CCAIDLE_TXEN, Enabled);

	// start clear channel assessment
	NRF_RADIO->TASKS_CCASTART = TRIGGER; // -> CCABUSY or CCAIDLE -> TXEN -> TXREADY
}

void Ieee802154Radio_RADIO_TIMER0_EGU0::startSend() {
	// handle sent packet on END event
	this->endAction = EndAction::ON_SENT;

	// start send operation
	NRF_RADIO->PACKETPTR = intptr_t(this->sendBuffer->packet);
	NRF_RADIO->TASKS_START = TRIGGER; // -> END
}

void Ieee802154Radio_RADIO_TIMER0_EGU0::startSendAck() {
	// don't do anything on END event
	this->endAction = EndAction::NOP;

	NRF_RADIO->PACKETPTR = intptr_t(this->ackPacket);
	NRF_RADIO->TASKS_START = TRIGGER; // -> END
}

void Ieee802154Radio_RADIO_TIMER0_EGU0::finishSend() {
	this->sendBuffer->finished = true;
	this->sendBuffer = nullptr;

	// sender is idle again
	this->sendState = SendState::IDLE;

	// signal to handle() that a send operation has finished
	NRF_EGU0->TASKS_TRIGGER[2] = TRIGGER;
}


void RADIO_IRQHandler() {
	Ieee802154Radio_RADIO_TIMER0_EGU0::instance->handleRadio();
}

void Ieee802154Radio_RADIO_TIMER0_EGU0::handleRadio() {
	// check if ready to receive (RxIdle state)
	if (NRF_RADIO->EVENTS_RXREADY) {
		NRF_RADIO->EVENTS_RXREADY = 0;

		// start receive
		if (this->receiverEnabled)
			startReceive(); // -> END

		// check if something to send (this is executed after start or send)
		if (this->sendState == SendState::IDLE) {
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
		if (this->endAction == EndAction::SEND_ACK) {
			// send ACK of last received packet overrides a normal send operation
			if (this->sendState != SendState::IDLE) {
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

		// get length of payload data (subtract 2 for crc)
		int length = this->receivePacket[0] - 2;

		// determine inter frame spacing duration
		this->ifsDuration = length <= maxSifsLength ? minSifsDuration : minLifsDuration;

		// frame control
		auto frameControl = ieee::FrameControl(receivePacket[1] | (receivePacket[2] << 8));

		// check if a previously sent packet awaits an ACK
		if (this->sendState == SendState::AWAIT_ACK) {
			// check if this is the ACK packet that we are waiting for (mac counter must match)
			auto frameType = frameControl & ieee::FrameControl::TYPE_MASK;
			if (frameType == ieee::FrameControl::TYPE_ACK && receivePacket[3] == this->sendBuffer->packet[3]) {
				// disable interrupts of ACK wait and backoff
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

		// capture timestamp of received packet (CC[1] is currently unused)
		if (RECEIVE_EXTRA_LENGTH >= 5) {
			NRF_TIMER0->TASKS_CAPTURE[1] = TRIGGER;
			uint32_t timestamp = NRF_TIMER0->CC[1];
			uint8_t *ts = &this->receivePacket[1 + length + 1];
			ts[0] = timestamp;
			ts[1] = timestamp >> 8;
			ts[2] = timestamp >> 16;
			ts[3] = timestamp >> 24;
		}

		// check if a node is interested in this packet and wants to handle ack
		bool pass = false;
		bool ack = false;
		this->ackPacket[1] = uint8_t(ieee::FrameControl::TYPE_ACK);
		bool receiverEnabled = false;
		for (auto &node : this->nodes) {
			// check if the node wants to receive the packet
			if (node.filterFlags != FilterFlags::NONE && node.filter(this->receivePacket)) {
				// find next receive buffer
				for (auto &buffer : node.receiveBuffers) {
					if (!buffer.finished) {
						bool passThis = true;

						// check if this context handles ack and the packet requests an ack
						if ((node.filterFlags & FilterFlags::HANDLE_ACK) != 0
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
								// skip length, frameControl and macCounter
								int i = 1 + 2 + 1;

								// pan id
								uint16_t panId = this->receivePacket[i] | (this->receivePacket[i + 1] << 8);
								i += 2;

								// skip destination address
								if ((frameControl & ieee::FrameControl::DESTINATION_ADDRESSING_LONG_FLAG) != 0)
									i += 8;
								else
									i += 2;

								// source address
								const uint8_t *sourceAddress = this->receivePacket + i;
								int len = (frameControl & ieee::FrameControl::SOURCE_ADDRESSING_LONG_FLAG) == 0 ? 2 : 8;

								// check if it is a data request command
								if (this->receivePacket[i + len] == uint8_t(ieee::Command::DATA_REQUEST)) {
									// check if there is a packet pending for this source and if yes, move to sendBuffers
									if (node.request(panId, sourceAddress, len)) {
										// set flag in ACK packet
										this->ackPacket[1] = uint8_t(ieee::FrameControl::TYPE_ACK | ieee::FrameControl::FRAME_PENDING);
									}

									// don't pass data request packet to node
									passThis = false;
								}
							}
						}
						if (passThis) {
							// assign received packet to buffer
							//buffer.set(this->receivePacket + 1, length + RECEIVE_EXTRA_LENGTH);
							int transferred = length + RECEIVE_EXTRA_LENGTH;
							std::copy(this->receivePacket + 1, this->receivePacket + 1 + transferred, buffer.dat);
							buffer.xferred = transferred;
							buffer.finished = true;
						}
						pass |= passThis;
						break;
					}
				}
			}

			// check if at least one node is "listening"
			if (!node.receiveBuffers.empty()) {
				auto it = node.receiveBuffers.end();
				--it;
				receiverEnabled |= !it->finished;
			}
		}
		this->receiverEnabled = receiverEnabled;

		// check if we need to send an ACK
		if (ack) {
			// note: clear channel assessment for a normal send operation may be in progress at this point. This means
			// that if the sender gets enabled (TXREADY), ACK has to override the normal send operation

			// copy frame sequence number to ACK packet
			this->ackPacket[3] = this->receivePacket[3];

			// send ACK on END event
			this->endAction = EndAction::SEND_ACK;
		}

		// check if a context is interested in the packet
		if (pass) {
			// signal to handle() that a received packet is ready
			NRF_EGU0->TASKS_TRIGGER[1] = TRIGGER;
		}
	}

	// check if packet was received with crc error
	if (NRF_RADIO->EVENTS_CRCERROR) {
		NRF_RADIO->EVENTS_CRCERROR = 0;

		// worst case assumption as we don't know how long the packet was
		this->ifsDuration = minLifsDuration;
	}

	// check if transfer has ended (receive or send)
	if (NRF_RADIO->EVENTS_END) {
		NRF_RADIO->EVENTS_END = 0;

		// capture time when last packet was sent or received (done automatically by PPI channel 27)
		//NRF_TIMER0->TASKS_CAPTURE[2] = TRIGGER;

		// disable after send (done automatically by SHORTS)
		//if (this->sendState != SendState::IDLE)
		//	NRF_RADIO->TASKS_DISABLE = TRIGGER; -> DISABLED

		// decide what to do next
		switch (this->endAction) {
		case EndAction::NOP:
			// do nothing
			break;
		case EndAction::RECEIVE:
			// continue receiving
			if (this->receiverEnabled)
				startReceive(); // -> END
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
		case EndAction::ON_SENT:
		{
			// determine interframe spacing duration
			uint8_t length = this->sendBuffer->packet[0];
			this->ifsDuration = length <= maxSifsLength ? minSifsDuration : minLifsDuration;

			// check if we have to wait for an ACK
			if (this->sendState == SendState::AWAIT_SENT_ACK) {
				// yes: set timer for ACK wait duration
				NRF_TIMER0->CC[1] = NRF_TIMER0->CC[2] + ackWaitDuration;
				NRF_TIMER0->EVENTS_COMPARE[1] = 0;
				NRF_TIMER0->INTENSET = N(TIMER_INTENSET_COMPARE1, Set); // -> COMPARE[1]

				// now we either receive an ACK packet or timeout
				this->sendState = SendState::AWAIT_ACK;
				this->receiverEnabled = true;
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
		if (this->stat == State::READY) {
			NRF_RADIO->TASKS_RXEN = TRIGGER; // -> RXREADY
		}
	}
}

void TIMER0_IRQHandler() {
	Ieee802154Radio_RADIO_TIMER0_EGU0::instance->handleTimer();
}

void Ieee802154Radio_RADIO_TIMER0_EGU0::handleTimer() {
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

		if (this->ackRetryCount < maxAckRetryCount) {
			// ack was not received: retry
			this->sendState = SendState::AWAIT_SENT_ACK;
			++this->ackRetryCount;
			startBackoff();
		} else {
//debug::set(debug::RED);
			// sent packet was not acknowledged: set result to failed, send state becomes idle
			this->sendBuffer->xferred = 0; // indicate failure
			finishSend();

			// todo: disable receiver (and set this->receiverenabled = false) when no node listens

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
		if (s && t >= this->ifsDuration) {
			// start clear channel assessment
			startClearChannelAssessment();
		} else {
			// backoff and try again
			backoff();
		}
	}
}

Ieee802154Radio_RADIO_TIMER0_EGU0 *Ieee802154Radio_RADIO_TIMER0_EGU0::instance;

} // namespace coco
