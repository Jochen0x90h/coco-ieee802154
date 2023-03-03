#pragma once

#include <coco/Ieee802154Radio.hpp>
#include <coco/platform/UsbHost_native.hpp>


namespace coco {

/**
 * Implementation of IEEE 802.15.4 radio using coco-usb.
 * 
 */
class Ieee802154Radio_usb : public Ieee802154Radio {
public:
	class Buffer;

	Ieee802154Radio_usb(UsbHost_native::Device &device);
	~Ieee802154Radio_usb() override;

	void start(int channel) override;
	void stop() override;


	/**
	 * Virtual node with own pan id and address. This can be used to use several networks and protocols at the same time
	 */
	class Node : public Ieee802154Radio::Node, public LinkedListNode {
		friend class Ieee802154Radio_usb;
		friend class Buffer;
	public:
		Node(Ieee802154Radio_usb &radio);
		~Node() override;

		void configure(uint16_t pan, uint64_t longAddress, uint16_t shortAddress, FilterFlags filterFlags) override;

		int getBufferCount() override;
		coco::Buffer &getBuffer(int index) override;

	protected:
		Coroutine receive();

		Ieee802154Radio_usb &radio;

		// usb endpoint to communicate with the radio device
		UsbHost_native::BulkEndpoint endpoint;
		
		// extra receive buffer that receives all packets and results for send operations
		UsbHost_native::BulkBuffer<PACKET_LENGTH> extraReceiveBuffer;

		uint64_t longAddress;
		uint16_t pan;
		uint16_t shortAddress;
		FilterFlags filterFlags;
		bool configureFlag = false;

		// list of receive buffers
		LinkedList2<Buffer> receiveBuffers;

		// list of send buffers
		LinkedList2<Buffer> sendBuffers;
	};

	class Buffer : public UsbHost_native::BulkBufferBase, public LinkedListNode2 {
		friend class Node;
	public:
		Buffer(Node &node) : UsbHost_native::BulkBufferBase(packet, PACKET_LENGTH, node.endpoint), node(node) {}
		~Buffer() override;

		bool start(Op op, int size) override;
		void cancel() override;
	
	protected:
		void setState(State state) override;

		uint8_t packet[PACKET_LENGTH];
		Node &node;
		
		enum ExtraState {
			IDLE,
			RECEIVING,
			WRITING,
			SENDING,
		};
		ExtraState extraState = ExtraState::IDLE;
	};

protected:
	Coroutine control();

	// usb device
	UsbHost_native::Device &device;
	
	// buffer for control transfers (stat, stop, configure)
	UsbHost_native::ControlBuffer<32> controlBuffer;
	Barrier<> controlBarrier;

	bool startStopFlag = false;
	int channel;

	LinkedList<Node> nodes;
};

} // namespace coco
