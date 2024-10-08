#include "Ieee802154Radio_usb.hpp"
#include <coco/debug.hpp>
#include <coco/ieee802154.hpp>
#include <coco/BufferWriter.hpp>


namespace coco {

namespace ieee = ieee802154;



// Ieee802154Radio_usb

Ieee802154Radio_usb::Ieee802154Radio_usb(coco::Buffer &controlBuffer, int headerSize)
	: Ieee802154Radio(State::DISABLED)
	, controlBuffer(controlBuffer), headerSize(headerSize)
{
	control();
}

Ieee802154Radio_usb::~Ieee802154Radio_usb() {
}

void Ieee802154Radio_usb::close() {
	this->startStopFlag = true;
	this->channel = 0;
	this->controlBarrier.doFirst();

	// set state of buffers to disabled and resume all coroutines waiting for a state change
	for (auto &node : this->nodes) {
		for (auto &buffer : node.buffers) {
			buffer.setDisabled();
		}
		node.st.set(State::DISABLED, Device::Events::ENTER_CLOSING | Device::Events::ENTER_DISABLED);
	}
	this->st.set(State::DISABLED, Device::Events::ENTER_CLOSING | Device::Events::ENTER_DISABLED);

}

void Ieee802154Radio_usb::open(int channel) {
	assert(channel >= 11 && channel <= 26);

	this->startStopFlag = true;
	this->channel = channel;
	this->controlBarrier.doFirst();

	// set state of buffers to ready and resume all coroutines waiting for a state change
	for (auto &node : this->nodes) {
		for (auto &buffer : node.buffers) {
			buffer.setReady(0);
		}
		node.st.set(State::READY, Device::Events::ENTER_OPENING | Device::Events::ENTER_READY);
	}
	this->st.set(State::READY, Device::Events::ENTER_OPENING | Device::Events::ENTER_READY);
}

Coroutine Ieee802154Radio_usb::control() {
	while (true) {
		auto &buffer = this->controlBuffer;
		co_await buffer.untilReady();

		// set header size of control buffer
		buffer.headerResize(sizeof(usb::Setup));

		while (buffer.ready()) {
			// check if at least one flag is set
			bool flag = this->startStopFlag;
			for (auto &node : this->nodes) {
				flag |= node.configureFlag;
			}

			// wait if no flag is set
			if (!flag)
				co_await controlBarrier.untilResumed();

			int size;
			if (this->startStopFlag) {
				// start/stop
				this->startStopFlag = false;
				auto request = this->channel != 0 ? Request::OPEN : Request::CLOSE;
				//buffer.setHeader<usb::Setup>({usb::RequestType::VENDOR_DEVICE_OUT, uint8_t(request), this->channel, 0, 0});
				buffer.header<usb::Setup>() = {usb::RequestType::VENDOR_DEVICE_OUT, uint8_t(request), this->channel, 0, 0};
				size = 0;
			} else {
				// configure one node
				uint16_t index = 1;
				for (auto &node : this->nodes) {
					if (node.configureFlag) {
						node.configureFlag = false;
						BufferWriter w(buffer.data(), buffer.capacity());
						w.u16L(node.pan);
						w.u64L(node.longAddress);
						w.u16L(node.shortAddress);
						w.e16L(node.filterFlags);
						size = w - buffer.begin();
						//buffer.setHeader<usb::Setup>({usb::RequestType::VENDOR_DEVICE_OUT, uint8_t(Request::CONFIGURE), 0, index, uint16_t(size)});
						buffer.header<usb::Setup>() = {usb::RequestType::VENDOR_DEVICE_OUT, uint8_t(Request::CONFIGURE), 0, index, uint16_t(size)};
						break;
					}
					++index;
				}
			}
			co_await buffer.write(size);
		}
	}
}


// Node

Ieee802154Radio_usb::Node::Node(Ieee802154Radio_usb &device, BufferDevice &wrappedDevice)
	: Ieee802154Radio::Node(device.st.state)
	, device(device), wrappedDevice(wrappedDevice)
{
	device.nodes.add(*this);
}

Ieee802154Radio_usb::Node::~Node() {
}

//StateTasks<const Device::State, Device::Events> &Ieee802154Radio_usb::Node::getStateTasks() {
//	return this->wrappedDevice.getStateTasks();
//}
/*
Device::State Ieee802154Radio_usb::Node::state() {
	return this->wrappedDevice.state();
}

Awaitable<Device::Condition> Ieee802154Radio_usb::Node::until(Condition condition) {
	return this->wrappedDevice.until(condition);
}*/

int Ieee802154Radio_usb::Node::getBufferCount() {
	//return this->wrappedDevice.getBufferCount();
	return this->buffers.count();
}

Buffer &Ieee802154Radio_usb::Node::getBuffer(int index) {
	//return this->wrappedDevice.getBuffer(index);
	return this->buffers.get(index);
}

void Ieee802154Radio_usb::Node::configure(uint16_t pan, uint64_t longAddress,
	uint16_t shortAddress, FilterFlags filterFlags)
{
	this->longAddress = longAddress;
	this->pan = pan;
	this->shortAddress = shortAddress;
	this->filterFlags = filterFlags;

	this->configureFlag = true;
	this->device.controlBarrier.doFirst();
}
/*
Coroutine Ieee802154Radio_usb::Node::stateTracker() {
	while (true) {
		co_await this->wrappedDevice.untilStateChanged();
	}
}
*/

// Buffer

Ieee802154Radio_usb::Buffer::Buffer(Node &node, coco::Buffer &wrappedBuffer)
	: coco::Buffer(wrappedBuffer.data() + 1, wrappedBuffer.capacity() - 1, wrappedBuffer.state()) // space for header size
	, node(node), wrappedBuffer(wrappedBuffer)
{
	node.buffers.add(*this);
	listen();
}

Ieee802154Radio_usb::Buffer::~Buffer() {
}

bool Ieee802154Radio_usb::Buffer::start(Op op) {
	if (this->st.state != State::READY) {
		assert(this->st.state != State::BUSY);
		return false;
	}

	// check if READ or WRITE flag is set
	assert((op & Op::READ_WRITE) != 0);

	this->op = op;
	if ((op & Op::WRITE) == 0) {
		// read
	} else {
		// write
		int headerSize = this->p.headerSize;
		this->wrappedBuffer[0] = headerSize;
		this->wrappedBuffer.resize(1 + this->p.size);

		// add to list until we receive the result of the send operation
		this->node.sendBuffers.add(*this);
	}
	bool result = this->wrappedBuffer.start(op);

	// in case setReady() was called from listen(), the state change is missed when the app call start() again
	if (result && this->st.state == Buffer::State::READY)
		setBusy();

	return result;
}

bool Ieee802154Radio_usb::Buffer::cancel() {
	if (this->st.state != State::BUSY)
		return false;

	// check if already cancelled
	if ((this->op & Op::CANCEL) != 0)
		return true;

	// set cancel indicator
	this->op |= Op::CANCEL;

	// cancel the USB transfer
	bool result = this->wrappedBuffer.cancel();

	// if cancel() failed, the USB transfer was already finished
	if (!result && (this->op & Op::WRITE) != 0) {
		// cancel write by sending mac counter
		auto data = this->wrappedBuffer.data();
		int headerSize = this->p.headerSize;//Capacity;
		uint8_t macCounter = this->wrappedBuffer[1 + headerSize + 2];
		data[0] = 0;
		data[1] = macCounter;
		this->wrappedBuffer.startWrite(2);
	}

	return true;
}

Coroutine Ieee802154Radio_usb::Buffer::listen() {
	while (true) {
		// wait for state change of the wrapped buffer
		co_await this->wrappedBuffer.untilStateChanged();

		auto state = this->wrappedBuffer.state();
		if (state == Buffer::State::READY) {
			// wrapped buffer became ready
			auto data = this->wrappedBuffer.data();
			int transferred = this->wrappedBuffer.size();
			//int headerSize = this->p.headerCapacity;

			if ((this->op & Op::CANCEL) != 0) {
				// cancelled (cancel() was called)
				if ((this->op & Op::WRITE) != 0) {
					// cancel write operation by sending the mac counter
					int headerSize = this->p.headerSize;//data[0];
					uint8_t macCounter = data[1 + headerSize + 2];
					data[0] = macCounter;
					this->wrappedBuffer.startWrite(1);

					// clear write flag so that cancel finishes when buffer becomes ready
					this->op = Op::CANCEL;
				} else {
					// cancel operation has finished
					setReady(0);
				}
			} else if ((this->op & Op::WRITE) == 0) {
				// read: received data from the radio
				if (transferred <= 2) {
					// data too small for a packet
					if (transferred == 2) {
						// received the result of a sent packet
						uint8_t macCounter = data[0];
						uint8_t radioTransferred = data[1];

						// search the buffer by mac counter
						for (auto &buffer : this->node.sendBuffers) {
							if (buffer[2] == macCounter) {
								buffer.remove2();
								buffer.setReady(radioTransferred);
								break;
							}
						}
					}

					// re-start read operation
					this->wrappedBuffer.start(this->op);
				} else {
					// received a packet
					int headerSize = data[0];
					this->p.headerSize = headerSize;
					this->p.size = transferred - 1;
					setReady();
				}
			} else {
				// write: do nothing and wait for result
			}
		} else if (state == Buffer::State::BUSY) {
			// wrapped buffer became busy
			setBusy();
			//setState(state);
		}
	}
}

} // namespace coco
