#include "Ieee802154Radio_usb.hpp"
#include <coco/debug.hpp>
#include <coco/ieee802154.hpp>
#include <coco/BufferWriter.hpp>


namespace coco {

namespace ieee = ieee802154;



// Ieee802154Radio_usb

Ieee802154Radio_usb::Ieee802154Radio_usb(coco::Buffer &controlBuffer, int headerSize)
	: controlBuffer(controlBuffer), headerSize(headerSize)
{
	control();
}

Ieee802154Radio_usb::~Ieee802154Radio_usb() {
}

void Ieee802154Radio_usb::start(int channel) {
	assert(channel >= 11 && channel <= 26);

	this->startStopFlag = true;
	this->channel = channel;
	this->controlBarrier.doFirst();
}

void Ieee802154Radio_usb::stop() {
	this->startStopFlag = true;
	this->channel = 0;
	this->controlBarrier.doFirst();
}

Coroutine Ieee802154Radio_usb::control() {
	while (true) {
		auto &buffer = this->controlBuffer;
		co_await buffer.untilReady();

		buffer.setHeader<usb::Setup>({usb::RequestType::VENDOR_DEVICE_OUT, uint8_t(Request::SET_HEADER_SIZE), this->headerSize, 0, 0});
		co_await buffer.write(0);

		while (buffer.ready()) {
			// check if at least one flag is set
			bool flag = this->startStopFlag;
			for (auto &node : this->nodes) {
				flag |= node.configureFlag;
			}

			// wait if no flag is set
			if (!flag)
				co_await controlBarrier.wait();

			int size;
			if (this->startStopFlag) {
				// start/stop
				this->startStopFlag = false;
				auto request = this->channel != 0 ? Request::START : Request::STOP;
				buffer.setHeader<usb::Setup>({usb::RequestType::VENDOR_DEVICE_OUT, uint8_t(request), this->channel, 0, 0});
				size = 0;
			} else {
				// configure one node
				uint16_t index = 1;
				for (auto &node : this->nodes) {
					if (node.configureFlag) {
						node.configureFlag = false;
						BufferWriter w(buffer);
						w.u16L(node.pan);
						w.u64L(node.longAddress);
						w.u16L(node.shortAddress);
						w.e16L(node.filterFlags);
						size = w - buffer.begin();
						buffer.setHeader<usb::Setup>({usb::RequestType::VENDOR_DEVICE_OUT, uint8_t(Request::CONFIGURE), 0, index, uint16_t(size)});
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
	: device(device), wrappedDevice(wrappedDevice)
{
	device.nodes.add(*this);
}

Ieee802154Radio_usb::Node::~Node() {
}

Device::State Ieee802154Radio_usb::Node::state() {
	return this->wrappedDevice.state();
}

Awaitable<> Ieee802154Radio_usb::Node::stateChange(int waitFlags) {
	return this->wrappedDevice.stateChange(waitFlags);
}

int Ieee802154Radio_usb::Node::getBufferCount() {
	return this->wrappedDevice.getBufferCount();
}

Buffer &Ieee802154Radio_usb::Node::getBuffer(int index) {
	return this->wrappedDevice.getBuffer(index);
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


// Buffer

Ieee802154Radio_usb::Buffer::Buffer(Node &node, coco::Buffer &wrappedBuffer)
	: BufferImpl(node.device.headerSize, wrappedBuffer.data() + node.device.headerSize, wrappedBuffer.capacity() - node.device.headerSize, wrappedBuffer.state())
	, node(node), wrappedBuffer(wrappedBuffer)
{
	listen();
}

Ieee802154Radio_usb::Buffer::~Buffer() {
}

bool Ieee802154Radio_usb::Buffer::start(Op op) {
	if (this->p.state != State::READY) {
		assert(this->p.state != State::BUSY);
		return false;
	}

	// check if READ or WRITE flag is set
	assert((op & Op::READ_WRITE) != 0);

	this->op = op;
	if ((op & Op::WRITE) == 0) {
		// read
	} else {
		// write
		int headerCapacity = this->p.headerCapacity;
		int headerSize = this->p.headerSize;
		auto data = this->wrappedBuffer.data();
		std::fill(data, data + headerCapacity - headerSize, 0);
		this->wrappedBuffer.resize(headerCapacity + this->p.size);

		// add to list until we receive the result of the send operation
		this->node.sendBuffers.add(*this);
	}
	bool result = this->wrappedBuffer.start(op);

	// in case setReady() was called from listen(), the state change is missed when the app call start() again
	if (result && this->p.state == Buffer::State::READY)
		setBusy();

	return result;
}

bool Ieee802154Radio_usb::Buffer::cancel() {
	if (this->p.state != State::BUSY)
		return false;

	// check if already cancelled
	if ((this->op & Op::ERASE) != 0)
		return true;

	// use ERASE flag as cancel indicator
	this->op |= Op::ERASE;
	if (!this->wrappedBuffer.cancel() && (this->op & Op::WRITE) != 0) {
		// cancel write by mac counter
		int headerSize = this->p.headerCapacity;
		uint8_t macCounter = this->wrappedBuffer[headerSize + 2];
		this->wrappedBuffer[0] = macCounter;
		this->wrappedBuffer.startWrite(1);
	}

	return true;
}

Coroutine Ieee802154Radio_usb::Buffer::listen() {
	while (true) {
		// wait for state change
		co_await this->wrappedBuffer.stateChange();

		auto state = this->wrappedBuffer.state();
		if (state == Buffer::State::READY) {
			// buffer became ready
			int transferred = this->wrappedBuffer.size();
			int headerSize = this->p.headerCapacity;

			if ((this->op & Op::ERASE) != 0) {
				// cancelled
				if ((this->op & Op::WRITE) != 0) {
					// cancel write by mac counter
					uint8_t macCounter = this->wrappedBuffer[headerSize + 2];
					this->wrappedBuffer[0] = macCounter;
					this->wrappedBuffer.startWrite(1);

					// clear write flag so that cancel finishes when buffer becomes ready
					this->op = Op::ERASE;
				} else {
					// cancel operation has finished
					setReady(0);
				}
			} else if ((this->op & Op::WRITE) == 0) {
				// received data from the radio
				if (transferred <= headerSize + 2) {
					// data too small for a packet
					if (transferred == 2) {
						// received the result of a sent packet
						auto d = this->wrappedBuffer.data();
						uint8_t macCounter = d[0];
						uint8_t radioTransferred = d[1];
						for (auto &buffer : this->node.sendBuffers) {
							int hs = buffer[0];
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
					setReady(transferred - headerSize);
				}
			} else {
				// write: do nothing and wait for result
			}
		} else {
			setState(state);
		}
	}
}

} // namespace coco
