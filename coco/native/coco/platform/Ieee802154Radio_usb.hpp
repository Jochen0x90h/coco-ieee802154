#pragma once

#include <coco/Ieee802154Radio.hpp>
#include <coco/platform/UsbHost_native.hpp>


namespace coco {

/**
	Implementation of IEEE 802.15.4 radio using coco-usb and an nRF52480 dongle running tools/RadioDevice.
*/
class Ieee802154Radio_usb : public Ieee802154Radio {
public:
	class Buffer;

	Ieee802154Radio_usb(coco::Buffer &controlBuffer, int headerSize = 1);
	~Ieee802154Radio_usb() override;

	void start(int channel) override;
	void stop() override;


	class Buffer;

	/**
		Virtual node with own pan id and address. This can be used to use several networks and protocols at the same time
	*/
	class Node : public IntrusiveListNode, public Ieee802154Radio::Node {
		friend class Ieee802154Radio_usb;
		friend class Buffer;
	public:
		Node(Ieee802154Radio_usb &device, BufferDevice &wrappedDevice);
		~Node() override;

		State state() override;
		[[nodiscard]] Awaitable<> stateChange(int waitFlags = -1) override;
		int getBufferCount() override;
		coco::Buffer &getBuffer(int index) override;

		void configure(uint16_t pan, uint64_t longAddress, uint16_t shortAddress, FilterFlags filterFlags) override;

	protected:
		Ieee802154Radio_usb &device;
		BufferDevice &wrappedDevice;

		uint64_t longAddress;
		uint16_t pan;
		uint16_t shortAddress;
		FilterFlags filterFlags;
		bool configureFlag = false;

		// list of receive buffers
		//LinkedList2<Buffer> receiveBuffers;

		// list of send buffers
		IntrusiveList2<Buffer> sendBuffers;
	};

	class Buffer : public BufferImpl, public IntrusiveListNode2 {
		friend class Node;
	public:
		Buffer(Node &node, coco::Buffer &wrappedBuffer);
		~Buffer() override;

		bool start(Op op) override;
		bool cancel() override;

	protected:
		Coroutine listen();

		Node &node;
		coco::Buffer &wrappedBuffer;

		Op op = Op::READ;
	};

protected:
	Coroutine control();

	// buffer for control transfers (stat, stop, configure)
	coco::Buffer &controlBuffer;
	Barrier<> controlBarrier;

	uint16_t headerSize;
	uint16_t channel;
	bool startStopFlag = false;

	IntrusiveList<Node> nodes;
};

} // namespace coco
