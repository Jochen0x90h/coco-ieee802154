#include <coco/debug.hpp>
#include <RadioSendTest.hpp>
#ifdef NATIVE
#include <iostream>
#include <iomanip>
#endif


/*
	This test sends a packet every second and sets the green led on success (ack received) and red led on error.
	Use RadioReceiveTest.cpp on a second device.
*/

int channel = 15;

// [0-1]: frame control, [2]: mac counter (50), [3-4]: pan id (0x0000), [5-6]: short address (0x1337)
uint8_t packetNoAck[] = {0x41, 0x88, 50, 0x00, 0x00, 0x37, 0x13};//, uint8_t(Ieee802154Radio::SendFlags::NONE)};

// packet with FrameControl::ACKNOWLEDGE_REQUEST
uint8_t packetAck[] = {0x61, 0x88, 50, 0x00, 0x00, 0x37, 0x13};//, uint8_t(Ieee802154Radio::SendFlags::NONE)};


// periodically send packets
Coroutine send(Loop &loop, Buffer &radioBuffer) {
	auto &packet = packetAck;
	while (true) {
#ifdef NATIVE
		std::cout << "Waiting radio device to become ready..." << std::endl;
#endif
		co_await radioBuffer.untilReady();
		while (radioBuffer.ready()) {
			co_await loop.sleep(900ms);
			debug::set(debug::BLACK);
			co_await loop.sleep(100ms);

			// send packet over the air
#ifdef NATIVE
			std::cout << "Send packet" << std::endl;
#endif
			co_await radioBuffer.writeArray(packet);
			int transferred = radioBuffer.size();

			// check for success
			bool success = transferred > 0;
			debug::set(success ? debug::GREEN : debug::RED);

			// increment mac counter
			packet[2]++;

			// change channel
			/*radio::stop();
			channel ^= 1;
			radio::start(channel);
			radio::enableReceiver(true);*/
		}
	}
}

// reply to received packets
Coroutine reply(Loop &loop, Buffer &radioBuffer) {
	auto &replyPacket = packetNoAck;
	while (true) {
		co_await radioBuffer.untilReady();
		while (radioBuffer.ready()) {
			// wait for receive packet
			debug::setBlue(true);
			co_await radioBuffer.read();
			debug::setBlue(false);

			// reply
			debug::setRed(true);
			co_await radioBuffer.writeArray(replyPacket);
			debug::setRed(false);
		}
	}
}

// for native, we also need to receive to get the results of the send operations. This is a limitation of the USB wrapper
#ifdef NATIVE
Coroutine receive(Loop &loop, Buffer &radioBuffer) {
	while (true) {
		co_await radioBuffer.untilReady();
		while (radioBuffer.ready()) {
			co_await radioBuffer.read();
		}
	}
}
#endif

int main() {
	drivers.node.configure(0, // pan id
		UINT64_C(0x0000133700001337), // long address
		0x1337, // short address
		Ieee802154Radio::FilterFlags::PASS_DEST_LONG | Ieee802154Radio::FilterFlags::PASS_DEST_SHORT | Ieee802154Radio::FilterFlags::HANDLE_ACK);
		//Ieee802154Radio::FilterFlags::PASS_ALL);

	// start radio
	drivers.radio.start(channel);

	send(drivers.loop, drivers.radioBuffer);
	//reply(drivers.loop, drivers.radioBuffer);

	// for native, we also need to receive to get the results of the send operations
#ifdef NATIVE
	receive(drivers.loop, drivers.radioReceiveBuffer);
#endif

	drivers.loop.run();
}
