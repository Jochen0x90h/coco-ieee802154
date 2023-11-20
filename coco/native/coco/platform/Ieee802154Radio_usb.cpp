#include "Ieee802154Radio_usb.hpp"
#include <coco/debug.hpp>
#include <coco/ieee802154.hpp>
#include <coco/BufferWriter.hpp>


namespace coco {

namespace ieee = ieee802154;



// Ieee802154Radio_usb

Ieee802154Radio_usb::Ieee802154Radio_usb(coco::Buffer &controlBuffer)
	: controlBuffer(controlBuffer)
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
	this->channel = -1;
	this->controlBarrier.doFirst();
}

Coroutine Ieee802154Radio_usb::control() {
	while (true) {
		auto &buffer = this->controlBuffer;
		co_await buffer.untilReady();
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
				auto request = this->channel >= 0 ? Request::START : Request::STOP;
				buffer.setHeader<usb::Setup>({usb::RequestType::VENDOR_DEVICE_OUT, uint8_t(request), uint16_t(this->channel), 0, 0});
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

Ieee802154Radio_usb::Node::Node(Ieee802154Radio_usb &radio, BufferDevice &device)
	: radio(radio), device(device)
{
	radio.nodes.add(*this);
}

Ieee802154Radio_usb::Node::~Node() {
}

Device::State Ieee802154Radio_usb::Node::state() {
	return this->device.state();
}

Awaitable<> Ieee802154Radio_usb::Node::stateChange(int waitFlags) {
	return this->device.stateChange(waitFlags);
}

int Ieee802154Radio_usb::Node::getBufferCount() {
	return this->device.getBufferCount();
}

Buffer &Ieee802154Radio_usb::Node::getBuffer(int index) {
	return this->device.getBuffer(index);
}

void Ieee802154Radio_usb::Node::configure(uint16_t pan, uint64_t longAddress,
	uint16_t shortAddress, FilterFlags filterFlags)
{
	this->longAddress = longAddress;
	this->pan = pan;
	this->shortAddress = shortAddress;
	this->filterFlags = filterFlags;

	this->configureFlag = true;
	this->radio.controlBarrier.doFirst();
}


// Buffer

Ieee802154Radio_usb::Buffer::Buffer(Node &node, coco::Buffer &buffer)
	: BufferImpl(MAX_HEADER_SIZE, buffer.data() + 1 + MAX_HEADER_SIZE, buffer.capacity() - 1 - MAX_HEADER_SIZE, buffer.state()), node(node), buffer(buffer)
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
		//this->xferred = size;
	} else {
		// write

		// move data
		int headerSize = this->p.headerSize;
		int size = this->p.size;
		uint8_t *d = this->buffer.data();
		d[0] = headerSize;
		uint8_t *src = headerData();
		uint8_t *dst = d + 1;
		std::memmove(dst, src, headerSize + size);
		this->buffer.resize(size);

		// add to list until we receive the result of the send operation
		this->node.sendBuffers.add(*this);
	}
	bool result = this->buffer.start(op);
	//if (result && this->stat == Buffer::State::READY)
	//	setBusy();
	return result;
}

bool Ieee802154Radio_usb::Buffer::cancel() {
	// todo: cancel by mac counter
	return this->buffer.cancel();
}

Coroutine Ieee802154Radio_usb::Buffer::listen() {
	while (true) {
		// wait for state change
		co_await this->buffer.stateChange();

		auto state = this->buffer.state();
		if (state == Buffer::State::READY) {
			// buffer became ready
			int transferred = this->buffer.size();

			if ((this->op & Op::WRITE) == 0) {
				// read
				int headerSize = this->buffer[0];
				int size = transferred - 1 - headerSize;
				if (headerSize >= 2 && size == 0) {
					// received mac counter and result of a sent packet
					auto h = this->p.data + 1 + headerSize - 2;
					uint8_t macCounter = h[0];
					uint8_t radioTransferred = h[1];
					for (auto &buffer : this->node.sendBuffers) {
						int hs = buffer[0];
						if (buffer[1 + hs + 2] == macCounter) {
							buffer.remove2();
							buffer.setReady(radioTransferred);
							break;
						}
					}

					// re-start read operation
					//this->buffer.start(this->xferred, Op::READ);
					this->buffer.startRead(this->p.size);
				} else {
					// received a packet

					// move data
					int hs = std::min(headerSize, 1 + MAX_HEADER_SIZE);
					int s = std::min(size, int(this->p.capacity));
					uint8_t *src = this->buffer.data() + 1 + headerSize - hs;
					uint8_t *dst = this->p.data - hs;
					std::memmove(dst, src, hs + s);

					setReady(s);
				}
			} else {
				// write: do nothing yet and wait for result
			}
		} else {
			setState(state);
		}
	}
}

} // namespace coco
