#include "Ieee802154Radio_usb.hpp"
#include <coco/debug.hpp>
#include <coco/ieee802154.hpp>
#include <coco/BufferWriter.hpp>


namespace coco {

namespace ieee = ieee802154;



// Ieee802154Radio_usb

Ieee802154Radio_usb::Ieee802154Radio_usb(UsbHost_native::Device &device)
	: device(device), controlBuffer(device)
{
	control();
}

Ieee802154Radio_usb::~Ieee802154Radio_usb() {
}

void Ieee802154Radio_usb::start(int channel) {
	assert(channel >= 11 && channel <= 26);

	this->startStopFlag = true;
	this->channel = channel;
	this->controlBarrier.resumeFirst();
}

void Ieee802154Radio_usb::stop() {
	this->startStopFlag = true;
	this->channel = -1;
	this->controlBarrier.resumeFirst();
}

Coroutine Ieee802154Radio_usb::control() {
	while (true) {
		co_await this->controlBuffer.untilReady();
		while (this->controlBuffer.ready()) {
			bool flag = this->startStopFlag;
			for (auto &node : this->nodes) {
				flag |= node.configureFlag;
			}
			if (!flag)
				co_await controlBarrier.wait();

			int size = 8;
			if (this->startStopFlag) {
				this->startStopFlag = false;
				auto request = this->channel >= 0 ? Request::START : Request::STOP;
				usb::setSetup(this->controlBuffer, {usb::RequestType::VENDOR_DEVICE_OUT, uint8_t(request), uint16_t(this->channel), 0, 0});
			} else {
				uint16_t index = 1;
				for (auto &node : this->nodes) {
					if (node.configureFlag) {
						node.configureFlag = false;
						usb::setSetup(this->controlBuffer, {usb::RequestType::VENDOR_DEVICE_OUT, uint8_t(Request::CONFIGURE), 0, index, 14});
						BufferWriter w(this->controlBuffer.data() + 8);
						w.u16L(node.pan);
						w.u64L(node.longAddress);
						w.u16L(node.shortAddress);
						w.e16L(node.filterFlags);
						size = 8 + 14;
						break;
					}
					++index;
				}
			}

			co_await this->controlBuffer.transfer(Buffer::Op::COMMAND8, size);
		}
	}
}


// Node

Ieee802154Radio_usb::Node::Node(Ieee802154Radio_usb &radio)
	: radio(radio), endpoint(radio.device, radio.nodes.count() + 1), extraReceiveBuffer(endpoint)
{
	radio.nodes.add(*this);
	
	// start coroutine
	receive();
}

Ieee802154Radio_usb::Node::~Node() {
}

void Ieee802154Radio_usb::Node::configure(uint16_t pan, uint64_t longAddress,
	uint16_t shortAddress, FilterFlags filterFlags)
{
	this->longAddress = longAddress;
	this->pan = pan;
	this->shortAddress = shortAddress;
	this->filterFlags = filterFlags;

	this->configureFlag = true;
	this->radio.controlBarrier.resumeFirst();
}

int Ieee802154Radio_usb::Node::getBufferCount() {
	// hide the extra receive buffer
	return this->endpoint.getBufferCount() - 1;
}

coco::Buffer &Ieee802154Radio_usb::Node::getBuffer(int index) {
	return this->endpoint.getBuffer(index + 1);
}

Coroutine Ieee802154Radio_usb::Node::receive() {
	while (true) {
		co_await this->extraReceiveBuffer.untilReady();
		while (this->extraReceiveBuffer.ready()) {
			co_await this->extraReceiveBuffer.read();

			int transferred = this->extraReceiveBuffer.transferred();

			if (transferred == 2) {
				// mac counter and result of a sent packet
				uint8_t mac = this->extraReceiveBuffer[0];
				uint8_t result = this->extraReceiveBuffer[1];
				for (auto &buffer : this->sendBuffers) {
					if (buffer.packet[2] == mac) {
						if (!result)
							buffer.completed(0);
						else
							buffer.setState(Buffer::State::READY);
						break;
					}
				}
			} else if (transferred > 2) {
				// received a packet
				for (auto &buffer : this->receiveBuffers) {
					buffer.copy(this->extraReceiveBuffer, transferred);
					buffer.completed(transferred);
					break;
				}
			}
		}
	}
}


// Buffer

Ieee802154Radio_usb::Buffer::~Buffer() {
}

bool Ieee802154Radio_usb::Buffer::start(Op op, int size) {
	if (this->p.state != State::READY || (op & Op::READ_WRITE) == 0) {
		assert(false);
		return false;
	}

	bool result = true;
	if ((op & Op::READ) != 0) {
		// read: only add to receive buffers and wait until something is received using the extra receive buffer
		this->node.receiveBuffers.add(*this);
		setState(State::BUSY);

		// set extra state
		this->extraState = ExtraState::RECEIVING;
	} else {
		// write
		result = UsbHost_native::BulkBufferBase::start(op, size);
		this->node.sendBuffers.add(*this);
		
		// set extra state
		this->extraState = ExtraState::WRITING;
	}
	return result;
}

void Ieee802154Radio_usb::Buffer::cancel() {
	if (this->extraState == ExtraState::RECEIVING) {
		// no extra state any more
		this->extraState = ExtraState::IDLE;

		// set state to cancelled
		setState(State::CANCELLED);
		
		// set state to ready
		setState(State::READY);		
	} else if (this->extraState == ExtraState::SENDING) {
		// set state to ready which is the "real" state so that start does not fail
		this->p.state = State::READY;

		// send cancel command (is the mac counter of the packet to cancel)
		this->packet[0] = this->packet[2];
		UsbHost_native::BulkBufferBase::start(Op::WRITE, 1);
		
		// no extra state any more
		this->extraState = ExtraState::IDLE;
		
		// set state to cancelled
		setState(State::CANCELLED);
	} else {
		UsbHost_native::BulkBufferBase::cancel();
	}
}

void Ieee802154Radio_usb::Buffer::setState(State state) {
	if (state == State::READY && this->extraState == ExtraState::WRITING) {
		// write operation to radio device has finished
		if (this->p.state == State::CANCELLED) {
			// set state to ready which is the "real" state so that start does not fail
			this->p.state = State::READY;

			// send cancel command in case the write operation has completed even if it was cancelled
			this->packet[0] = this->packet[2];
			UsbHost_native::BulkBufferBase::start(Op::WRITE, 1);

			// no extra state any more
			this->extraState = ExtraState::IDLE;

			// stay in cancelled state
		} else {
			// now the radio is sending and we need to wait for the result (gets received in the separate receive buffer)
			this->extraState = ExtraState::SENDING;
		}
	} else {
		if (this->extraState == ExtraState::IDLE || this->extraState == ExtraState::RECEIVING) {
			// remove from receive or send buffer list
			if (state == State::READY)
				remove2();
			UsbHost_native::BulkBufferBase::setState(state);
		}
	}
}

} // namespace coco
