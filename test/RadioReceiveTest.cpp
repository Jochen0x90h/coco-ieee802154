#include <coco/debug.hpp>
#include <RadioReceiveTest.hpp>
#ifdef NATIVE
#include <iostream>
#include <iomanip>
#endif

/*
	This test receives packets and outputs them to the console (native) or sets the debug led color dependent on the packet length.
	ACK is sent when requested, therefore can be used as receiver for RadioSendTest.cpp
*/

// receive packets
Coroutine receive(Loop &loop, Buffer &radioBuffer) {
	while (true) {
#ifdef NATIVE
		std::cout << "Waiting radio device to become ready..." << std::endl;
#endif
		co_await radioBuffer.untilReady();
#ifdef NATIVE
		std::cout << "Waiting for IEEE 802.15.4 packets..." << std::endl;
#endif
		while (radioBuffer.ready()) {
			// wait for receive packet
			co_await radioBuffer.read();
			int transferred = radioBuffer.size();

#ifdef NATIVE
			std::cout << std::dec << '(' << transferred << ") ";
			for (int i = 0; i < transferred; ++i) {
				if (i != 0)
					std::cout << ", ";
				std::cout << std::hex << std::setw(2) << std::setfill('0') << int(radioBuffer[i]);
			}
			std::cout << std::endl;
#else
			if (transferred == 0)
				debug::toggleRed();
			else if (transferred < 10)
				debug::toggleBlue();
			else
				debug::toggleGreen();
#endif
		}
	}
}


int main() {
	drivers.node.configure(0, UINT64_C(0x0000133700001337), 0x1337,
		Ieee802154Radio::FilterFlags::PASS_DEST_LONG | Ieee802154Radio::FilterFlags::PASS_DEST_SHORT | Ieee802154Radio::FilterFlags::HANDLE_ACK);
//		Ieee802154Radio::FilterFlags::PASS_ALL);

	// start radio
	drivers.radio.start(15);

	receive(drivers.loop, drivers.radioBuffer);

	drivers.loop.run();
}
