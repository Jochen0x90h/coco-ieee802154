#pragma once

#include <coco/Ieee802154Radio.hpp>
#include <coco/PseudoRandom.hpp>
#include <coco/IntrusiveList.hpp>
#include <coco/InterruptQueue.hpp>
#include <coco/platform/Loop_Queue.hpp>
#include <coco/platform/nvic.hpp>


namespace coco {

/// @brief Implementation of IEEE 802.15.4 radio using RADIO and TIMER0.
/// Supports multiple virtual nodes each with its own pan id and address. This can be used to use several networks and
/// protocols at the same time.
///
/// Resources:
///   NRF_RADIO
///   NRF_TIMER0
///     CC[0]: ack turnaround timeout, minimum time between received packet and sending ack
///     CC[1]: ack wait timeout, maximum time to wait for receiving ack after sending a packet
///     CC[2]: time of last packet received or sent
///     CC[3]: backoff timeout
///   NRF_PPI
///     CH[27]: RADIO.EVENTS_END -> TIMER0.TASKS_CAPTURE[2]
///
/// Glossary:
///   CCA: Clear Channel Assessment (-> energy and/or carrier detection)
///   ED: Energy Detection on a radio channel
///   RFD: Reduced-function defice (can only talk to -> FFDs)
///   FFD: Full-function device
///   MLME: MAC Layer Management Entity, management service of the MAC layer
///   MCPS: MAC Common Part Layer, data transport service of the MAC layer
///   PIB:  PAN Information Base
///
///  References:
///    https://www.prismmodelchecker.org/casestudies/zigbee.php
///
/// Bugs:
///   No timeout for send() with SendFlags::AWAIT_DATA_REQUEST, i.e. waits forever
class Ieee802154Radio_RADIO_TIMER0 : public Ieee802154Radio {
public:
    Ieee802154Radio_RADIO_TIMER0(Loop_Queue &loop);
    ~Ieee802154Radio_RADIO_TIMER0() override;

    // Device methods
    void close() override;

    // Ieee802154Radio methods
    void open(int channel) override;


    class Buffer;

    /// @brief Virtual node with own pan id and address.
    /// This can be used to participate in several networks at the same time
    class Node : public Ieee802154Radio::Node, public IntrusiveListNode {
        friend class Ieee802154Radio_RADIO_TIMER0;
        friend class Buffer;
    public:
        Node(Ieee802154Radio_RADIO_TIMER0 &device);
        ~Node() override;

        // BufferDevice methods
        int getBufferCount() override;
        coco::Buffer &getBuffer(int index) override;

        // Ieee802154Radio::Node methods
        void configure(uint16_t pan, uint64_t longAddress, uint16_t shortAddress, FilterFlags filterFlags) override;

    protected:
        // check if this virutal node is interested in the packet
        bool filter(const uint8_t *data) const;

        // check if a packet for given pan id and destination address is pending and move it from requestBuffers to
        // sendBuffers (called on receive of data request in the interrupt handler)
        bool request(uint16_t panId, const uint8_t *destinationAddress, int addressLength);

        Ieee802154Radio_RADIO_TIMER0 &device_;

        uint64_t longAddress_;
        uint16_t pan_;
        uint16_t shortAddress_;
        FilterFlags filterFlags_;

        // list of buffers
        IntrusiveList<Buffer> buffers_;

        // list of active receive buffers
        InterruptQueue<Buffer> receiveBuffers_;

        // list of buffers that can be sent on data request
        InterruptQueue<Buffer> requestBuffers_;
    };

    /// @brief Buffer for transferring data over the radio.
    /// Derives from IntrusiveListNode for the list of buffers and Loop_Queue::Handler to be notified from the event loop
    class Buffer : public coco::Buffer, public IntrusiveListNode, public Loop_Queue::Handler {
        friend class Ieee802154Radio_RADIO_TIMER0;
    public:
        Buffer(Node &node);
        ~Buffer() override;

        // Buffer methods
        bool start(Op op) override;
        bool cancel() override;

    protected:
        void handle() override;

        Node &node_;

        // space for length byte of send packets when no header is present
        //uint8_t length;
        uint8_t buffer_[BUFFER_SIZE];

        enum class Mode {
            // is in node.receiveTransfers
            RECEIVE,

            // is in node.requestBuffers
            REQUEST,

            // is in device.sendTransfers
            SEND,
        };
        Mode mode_;
    };

    void RADIO_IRQHandler();
    void TIMER0_IRQHandler();

protected:
    // start receiving packets (enable base band decoder)
    void startReceive();

    // select a packet for the next send operation
    void selectForSend();

    // prepare a packet for the next send operation
    void prepareForSend(Buffer &buffer);

    // start backoff for csma-ca
    void startBackoff();

    // repeat backoff for csma-ca
    void backoff();

    // start clear channel assessment
    void startClearChannelAssessment();

    // start to send a packet
    void startSend();

    // start to send an ack packet
    void startSendAck();

    // finish send operation
    void finishSend(bool success);

    // interrupt handlers
    void handleRadio();
    void handleTimer();


    Loop_Queue &loop_;

    // list of virtual nodes
    IntrusiveList<Node> nodes_;

    // receiver is enabled when start() gets called and disabled when stop() gets called
    bool receiverEnabled_ = false;

    // receive packet, gets copied to first receiveBuffer of each channel if it passes the filter
    uint8_t receivePacket_[1 + MAX_PAYLOAD_SIZE + 1]; // +1 for length and for link quality indicator

    // duration of inter frame spacing (depends on received packet length)
    uint32_t ifsDuration_ = 0;

    // list of active send transfers
    InterruptQueue<Buffer> sendBuffers_;
    uint8_t ackPacket_[4] = {5, 0x02, 0x00, 0};

    enum SendState : uint8_t {
        IDLE,

        // wait until send operation is done
        AWAIT_SENT,

        // wait for send and then ACK
        AWAIT_SENT_ACK,

        // wait for an ACK from the destination
        AWAIT_ACK,
    };
    SendState sendState_ = SendState::IDLE;

    // send retries when expecting ack
    int ackRetryCount_;

    // send backoff
    int backoffExponent_;
    int backoffCount_;

    // pseudo random number generator for backoff time
    XorShiftRandom random_;

    // action to take on END event (both receive and send)
    enum class EndAction {
        NOP,
        RECEIVE,
        SEND_ACK,
        //SEND_REQUESTED,
        ON_SENT
    };
    EndAction endAction_;
};

} // namespace coco
