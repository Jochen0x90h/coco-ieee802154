#pragma once

#include <coco/Ieee802154Radio.hpp>
#include <coco/platform/UsbHost_native.hpp>


namespace coco {

/**
 * Implementation of IEEE 802.15.4 radio using coco-usb and an nRF52480 dongle running tools/RadioDevice.
 */
class Ieee802154Radio_usb : public Ieee802154Radio {
public:
	class Buffer;

	Ieee802154Radio_usb(coco::Buffer &controlBuffer, int headerSize = 1);
	~Ieee802154Radio_usb() override;

	// Device methods
	void close() override;

	// Ieee802154Radio methods
	void open(int channel) override;


	class Buffer;

	/**
	 * Virtual node with own pan id and address. This can be used to use several networks and protocols at the same time
	 */
	class Node : public Ieee802154Radio::Node, public IntrusiveListNode {
		friend class Ieee802154Radio_usb;
		friend class Buffer;
	public:
		Node(Ieee802154Radio_usb &device, BufferDevice &wrappedDevice);
		~Node() override;

		// Device methods
		//StateTasks<const State, Events> &getStateTasks() override;
		//State state() override;
		//[[nodiscard]] Awaitable<Condition> until(Condition condition) override;

		// BufferDevice methods
		int getBufferCount() override;
		coco::Buffer &getBuffer(int index) override;

		// Ieee802154Radio::Node methods
		void configure(uint16_t pan, uint64_t longAddress, uint16_t shortAddress, FilterFlags filterFlags) override;

	protected:
		//Coroutine stateTracker();

		Ieee802154Radio_usb &device;
		BufferDevice &wrappedDevice;

		uint64_t longAddress;
		uint16_t pan;
		uint16_t shortAddress;
		FilterFlags filterFlags;
		bool configureFlag = false;

		// list of buffers
		IntrusiveList<Buffer> buffers;

		// list of receive buffers
		//LinkedList2<Buffer> receiveBuffers;

		// list of send buffers
		IntrusiveList2<Buffer> sendBuffers;
	};

	/**
	 * Buffer for transferring data via USB over the radio.
	 * Derives from IntrusiveListNode for the list of buffers and IntrusiveListNode2 for list of send buffers
	 */
	class Buffer : public coco::Buffer, public IntrusiveListNode, public IntrusiveListNode2 {
		friend class Ieee802154Radio_usb;
		friend class Node;
	public:
		Buffer(Node &node, coco::Buffer &wrappedBuffer);
		~Buffer() override;

		bool start(Op op) override;
		bool cancel() override;

	protected:
		// listen on state changes of the wrapped buffer
		Coroutine listen();

		Node &node;

		// wrapped USB buffer
		coco::Buffer &wrappedBuffer;

		Op op = Op::READ;
	};

protected:
	Coroutine control();

	using Device::st;

	// buffer for control transfers (stat, stop, configure)
	coco::Buffer &controlBuffer;
	Barrier<> controlBarrier;

	uint16_t headerSize;
	uint16_t channel;
	bool startStopFlag = false;

	IntrusiveList<Node> nodes;
};

} // namespace coco
