#pragma once

#include <coco/Ieee802154Radio.hpp>
#include <coco/BufferImpl.hpp>
#include <coco/PseudoRandom.hpp>
#include <coco/platform/Loop_RTC0.hpp>


namespace coco {

extern "C" {
	void RADIO_IRQHandler();
	void TIMER0_IRQHandler();
}

/**
	Implementation of IEEE 802.15.4 radio using RADIO and TIMER0.
	Supports multiple virtual nodes each with its own pan id and address. This can be used to use several networks and
	protocols at the same time.

	Resources:
		NRF_RADIO
		NRF_TIMER0
 			CC[0]: ack turnaround timeout, minimum time between received packet and sending ack
 			CC[1]: ack wait timeout, maximum time to wait for receiving ack after sending a packet
 			CC[2]: time of last packet received or sent
			CC[3]: backoff timeout
		NRF_PPI
 			CH[27]: RADIO->EVENTS_END -> TIMER0->TASKS_CAPTURE[2]
		NRF_EGU0
 			TRIGGER[0]: energy detection
 			TRIGGER[1]: receive queue
			TRIGGER[2]: send queue

	Glossary:
		CCA: Clear Channel Assessment (-> ED and/or carrier detection)
		ED: Energy Detection on a radio channel
		RFD: Reduced-function defice (can only talk to -> FFDs)
		FFD: Full-function device
		MLME: MAC Layer Management Entity, management service of the MAC layer
		MCPS: MAC Common Part Layer, data transport service of the MAC layer
		PIB:  PAN Information Base

	References:
		https://www.prismmodelchecker.org/casestudies/zigbee.php

	Bugs:
		No timeout for send() with SendFlags::AWAIT_DATA_REQUEST, i.e. waits forever
*/
class Ieee802154Radio_RADIO_TIMER0_EGU0 : public Ieee802154Radio, public Loop_RTC0::Handler {
	friend void RADIO_IRQHandler();
	friend void TIMER0_IRQHandler();
public:
	Ieee802154Radio_RADIO_TIMER0_EGU0(Loop_RTC0 &loop);
	~Ieee802154Radio_RADIO_TIMER0_EGU0() override;

	void start(int channel) override;
	void stop() override;


	class Buffer;

	/**
		Virtual node with own pan id and address. This can be used to use several networks and protocols at the same time
	*/
	class Node : public Ieee802154Radio::Node, public LinkedListNode {
		friend class Ieee802154Radio_RADIO_TIMER0_EGU0;
		friend class Buffer;
	public:
		Node(Ieee802154Radio_RADIO_TIMER0_EGU0 &radio);
		~Node() override;

		State state() override;
		[[nodiscard]] Awaitable<> stateChange(int waitFlags = -1) override;
		int getBufferCount() override;
		coco::Buffer &getBuffer(int index) override;

		void configure(uint16_t pan, uint64_t longAddress, uint16_t shortAddress, FilterFlags filterFlags) override;

	protected:
		// check if this virutal node is interested in the packet
		bool filter(const uint8_t *data) const;

		// check if a packet for given pan id and destination address is pending and move it from requestBuffers to
		// sendBuffers (called on receive of data request in the interrupt handler)
		bool request(uint16_t panId, const uint8_t *destinationAddress, int addressLength);

		Ieee802154Radio_RADIO_TIMER0_EGU0 &radio;

		uint64_t longAddress;
		uint16_t pan;
		uint16_t shortAddress;
		FilterFlags filterFlags;

		// list of buffers
		LinkedList<Buffer> buffers;

		// list of receive buffers
		LinkedList2<Buffer> receiveBuffers;

		// list of buffers that can be sent on data request
		LinkedList2<Buffer> requestBuffers;
	};

	class Buffer : public BufferImpl, public LinkedListNode, public LinkedListNode2 {
		friend class Ieee802154Radio_RADIO_TIMER0_EGU0;
	public:
		Buffer(Node &node);
		~Buffer() override;

		void cancel() override;

	protected:
		bool startInternal(int size, Op op) override;

		// first byte contains the length of the payload plus 2 for crc
		uint8_t packet[1 + PACKET_LENGTH];
		Node &node;

		bool finished;
	};

protected:
	void handle() override;

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
	void finishSend();

	// interrupt handlers
	void handleRadio();
	void handleTimer();

	static Ieee802154Radio_RADIO_TIMER0_EGU0 *instance;

	// radio state
	State stat = State::DISABLED;

	CoroutineTaskList<> stateTasks;

	// list of virtual nodes
	LinkedList<Node> nodes;

	// receiver is enabled if at least one receive buffer is queued (in Channel::receiveBuffers) or when waiting for ACK
	bool receiverEnabled = false;

	// receive packet, gets copied to first receiveBuffer of each channel if it passes the filter
	uint8_t receivePacket[1 + PACKET_LENGTH];

	// duration of inter frame spacing (depends on received packet length)
	uint32_t ifsDuration = 0;

	// list of buffers that are ready to send
	LinkedList2<Buffer> sendBuffers;
	Buffer *sendBuffer = nullptr;
	uint8_t ackPacket[4] = {5, 0x02, 0x00, 0};

	enum SendState : uint8_t {
		IDLE,

		// wait until send operation is done
		AWAIT_SENT,

		// wait for send and then ACK
		AWAIT_SENT_ACK,

		// wait for an ACK from the destination
		AWAIT_ACK,
	};
	SendState sendState = SendState::IDLE;

	// send retries when expecting ack
	int ackRetryCount;

	// send backoff
	int backoffExponent;
	int backoffCount;

	// pseudo random generator for backoff time
	XorShiftRandom random;

	// action to take on END event (both receive and send)
	enum class EndAction {
		NOP,
		RECEIVE,
		SEND_ACK,
		//SEND_REQUESTED,
		ON_SENT
	};
	EndAction endAction;
};

} // namespace coco
