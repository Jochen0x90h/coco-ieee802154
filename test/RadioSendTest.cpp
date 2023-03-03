#include <coco/debug.hpp>
#include <RadioSendTest.hpp>
#ifdef NATIVE
#include <iostream>
#include <iomanip>
#endif


/*
	This test sends a packet every second and sets the green led on success (ack received) and red led on error
*/

int channel = 15;

// [0-1]: frame control, [2]: mac counter, [3-4]: pan id, [5-6]: short address)
uint8_t packetNoAck[] = {0x41, 0x88, 50, 0x00, 0x00, 0x37, 0x13, uint8_t(Ieee802154Radio::SendFlags::NONE)};

// packet with with FrameControl::ACKNOWLEDGE_REQUEST
uint8_t packetAck[] = {0x61, 0x88, 50, 0x00, 0x00, 0x37, 0x13, uint8_t(Ieee802154Radio::SendFlags::NONE)};


// periodically send packets
Coroutine send(Loop &loop, Buffer &radioBuffer) {
	auto &packet = packetAck;
	while (true) {
#ifdef NATIVE
		std::cout << "Waiting radio device to become ready..." << std::endl;
#endif
		co_await radioBuffer.untilReady();
		while (radioBuffer.ready()) {
			co_await loop.sleep(1s);

			// send over the air and increment mac counter
			co_await radioBuffer.writeArray(packet);
			int transferred = radioBuffer.transferred();

			// increment mac counter
			packet[2]++;

			bool success = transferred > 0;
			debug::setRed(!success);
			debug::setGreen(success);

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
	debug::init();
	Drivers drivers;

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
