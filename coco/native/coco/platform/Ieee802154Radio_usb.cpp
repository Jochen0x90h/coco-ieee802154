#include "Ieee802154Radio_usb.hpp"
#include <coco/debug.hpp>
#include <coco/ieee802154.hpp>
#include <coco/BufferWriter.hpp>


namespace coco {

namespace ieee = ieee802154;



// Ieee802154Radio_usb

Ieee802154Radio_usb::Ieee802154Radio_usb(coco::Buffer &controlBuffer)
    : Ieee802154Radio(State::DISABLED)
    , controlBuffer_(controlBuffer)
{
    control();
}

Ieee802154Radio_usb::~Ieee802154Radio_usb() {
}

void Ieee802154Radio_usb::close() {
    startStopFlag_ = true;
    channel_ = 0;
    controlBarrier_.doFirst();

    // set state
    st.set(State::DISABLED);

    // iterate over nodes
    for (auto &node : nodes_) {
        // set state
        node.st.set(State::DISABLED);

        // disable buffers
        for (auto &buffer : node.buffers_) {
            buffer.setDisabled();
        }

        // resume all coroutines waiting for state change
        node.st.notify(Device::Events::ENTER_CLOSING | Device::Events::ENTER_DISABLED);
    }

    // resume all coroutines waiting for state change
    st.notify(Device::Events::ENTER_CLOSING | Device::Events::ENTER_DISABLED);
}

void Ieee802154Radio_usb::open(int channel) {
    assert(channel >= 11 && channel <= 26);

    startStopFlag_ = true;
    channel_ = channel;
    controlBarrier_.doFirst();

    // set state
    st.set(State::READY);

    // iterate over nodes
    for (auto &node : nodes_) {
        // set state
        node.st.set(State::READY);

        for (auto &buffer : node.buffers_) {
            buffer.setReady(0);
        }

        node.st.notify(Device::Events::ENTER_OPENING | Device::Events::ENTER_READY);
    }

    st.notify(Device::Events::ENTER_OPENING | Device::Events::ENTER_READY);
}

Coroutine Ieee802154Radio_usb::control() {
    while (true) {
        auto &buffer = controlBuffer_;
        co_await buffer.untilReady();

        while (buffer.ready()) {
            // check if at least one flag is set
            bool flag = startStopFlag_;
            for (auto &node : nodes_) {
                flag |= node.configureFlag_;
            }

            // wait if no flag is set
            if (!flag)
                co_await controlBarrier_.untilResumed();

            int size;
            if (startStopFlag_) {
                // start/stop
                startStopFlag_ = false;
                auto request = channel_ != 0 ? Request::OPEN : Request::CLOSE;
                buffer.header<usb::Setup>() = {usb::RequestType::VENDOR_DEVICE_OUT, uint8_t(request), channel_, 0, 0};
                size = 0;
            } else {
                // configure one node
                uint16_t index = 1;
                for (auto &node : nodes_) {
                    if (node.configureFlag_) {
                        node.configureFlag_ = false;
                        BufferWriter w(buffer.data(), buffer.capacity());
                        w.u16L(node.pan_);
                        w.u64L(node.longAddress_);
                        w.u16L(node.shortAddress_);
                        w.e16L(node.filterFlags_);
                        size = w - buffer.begin();
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


// Ieee802154Radio_usb::Node

Ieee802154Radio_usb::Node::Node(Ieee802154Radio_usb &device, BufferDevice &wrappedDevice)
    : Ieee802154Radio::Node(device.st.state)
    , device_(device), wrappedDevice_(wrappedDevice)
{
    device.nodes_.add(*this);
}

Ieee802154Radio_usb::Node::~Node() {
}

int Ieee802154Radio_usb::Node::getBufferCount() {
    return buffers_.count();
}

Buffer &Ieee802154Radio_usb::Node::getBuffer(int index) {
    return buffers_.get(index);
}

void Ieee802154Radio_usb::Node::configure(uint16_t pan, uint64_t longAddress,
    uint16_t shortAddress, FilterFlags filterFlags)
{
    longAddress_ = longAddress;
    pan_ = pan;
    shortAddress_ = shortAddress;
    filterFlags_ = filterFlags;

    configureFlag_ = true;
    device_.controlBarrier_.doFirst();
}


// Ieee802154Radio_usb::Buffer

Ieee802154Radio_usb::Buffer::Buffer(Node &node, coco::Buffer &wrappedBuffer)
    : coco::Buffer(wrappedBuffer.data(), Ieee802154Radio_usb::HEADER_SIZE, 0, wrappedBuffer.capacity() - Ieee802154Radio_usb::HEADER_SIZE, wrappedBuffer.state())
    , node_(node), wrappedBuffer_(wrappedBuffer)
{
    node.buffers_.add(*this);
    listen();
}

Ieee802154Radio_usb::Buffer::~Buffer() {
}

bool Ieee802154Radio_usb::Buffer::start(Op op) {
    if (st.state != State::READY) {
        assert(st.state != State::BUSY);
        return false;
    }

    // check if READ or WRITE flag is set
    assert((op & Op::READ_WRITE) != 0);

    const int headerSize = Ieee802154Radio::HEADER_SIZE;
    op_ = op;
    if ((op & Op::WRITE) == 0) {
        // read
    } else {
        // write

        // set size wrapped buffer
        wrappedBuffer_.resize(headerSize + size_);

        // add to list until we receive the result of the send operation
        node_.sendBuffers_.add(*this);
    }

    // start USB transfer
    bool result = wrappedBuffer_.start(op);

    // in case setReady() was called from listen(), the state change is missed when the app call start() again
    if (result && st.state == Buffer::State::READY)
        setBusy();

    return result;
}

bool Ieee802154Radio_usb::Buffer::cancel() {
    if (st.state != State::BUSY)
        return false;

    // check if already cancelled
    if ((op_ & Op::CANCEL) != 0)
        return true;

    const int headerSize = Ieee802154Radio::HEADER_SIZE;

    // set cancel indicator
    op_ |= Op::CANCEL;

    // cancel the USB transfer
    bool result = wrappedBuffer_.cancel();

    // if cancel() failed, the USB transfer was already finished
    if (!result && (op_ & Op::WRITE) != 0) {
        // cancel write by sending mac counter
        uint8_t macCounter = wrappedBuffer_[headerSize + 2];
        wrappedBuffer_[0] = macCounter;
        wrappedBuffer_.startWrite(1);
    }

    return true;
}

Coroutine Ieee802154Radio_usb::Buffer::listen() {
    const int headerSize = Ieee802154Radio::HEADER_SIZE;
    while (true) {
        // wait for state change of the wrapped buffer
        co_await wrappedBuffer_.untilStateChanged();

        auto state = wrappedBuffer_.state();
        if (state == Buffer::State::READY) {
            // wrapped buffer became ready
            auto data = wrappedBuffer_.data();
            int transferred = wrappedBuffer_.size();

            if ((op_ & Op::CANCEL) != 0) {
                // cancelled (cancel() was called)
                if ((op_ & Op::WRITE) != 0) {
                    // cancel write operation by sending the mac counter
                    uint8_t macCounter = data[headerSize + 2];
                    data[0] = macCounter;
                    wrappedBuffer_.startWrite(1);

                    // clear write flag so that cancel finishes when buffer becomes ready
                    op_ = Op::CANCEL;
                } else {
                    // cancel operation has finished
                    setReady(0);
                }
            } else if ((op_ & Op::WRITE) == 0) {
                // read: received data from the radio
                if (transferred < headerSize + 2) {
                    // data too small for a packet (must contain at least 2 bytes for frame control)
                    if (transferred == 2) {
                        // received the result of a sent packet
                        uint8_t macCounter = data[0];
                        uint8_t radioTransferred = data[1];

                        // search the sent buffer by mac counter
                        for (auto &buffer : node_.sendBuffers_) {
                            if (buffer[2] == macCounter) {
                                buffer.remove2();

                                // notify application that sent buffer is ready
                                buffer.setReady(radioTransferred);
                                break;
                            }
                        }
                    }

                    // re-start read operation
                    wrappedBuffer_.start(op_);
                } else {
                    // received a packet
                    setReady(transferred - headerSize);
                }
            } else {
                // write: do nothing and wait for result
            }
        } else if (state == Buffer::State::BUSY) {
            // wrapped buffer became busy
            setBusy();
        }
    }
}

} // namespace coco
