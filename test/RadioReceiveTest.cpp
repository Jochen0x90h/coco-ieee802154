#include <coco/debug.hpp>
#include <RadioReceiveTest.hpp>
#ifdef NATIVE
#include <iostream>
#include <iomanip>
#endif

/*
	This test receives packets and sets the debug led color dependent on the packet type
*/

// receive packets
Coroutine receive(Loop &loop, Ieee802154Radio::Node &node) {
	Buffer &buffer = node.getBuffer(0);
	while (true) {
		co_await buffer.untilReady();
		while (buffer.ready()) {
			// wait for receive packet
			co_await buffer.read();
			int transferred = buffer.transferred();

#ifdef NATIVE
			std::cout << std::dec << '(' << transferred << ") ";
			for (int i = 0; i < transferred; ++i) {
				if (i != 0)
					std::cout << ", ";
				std::cout << std::hex << std::setw(2) << std::setfill('0') << int(buffer[i]);
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
	debug::init();
	Drivers drivers;
	
	drivers.node.configure(0, UINT64_C(0x0000133700001337), 1337,
		Ieee802154Radio::FilterFlags::PASS_DEST_LONG | Ieee802154Radio::FilterFlags::PASS_DEST_SHORT | Ieee802154Radio::FilterFlags::HANDLE_ACK);
//		Ieee802154Radio::FilterFlags::PASS_ALL);

	// start radio
	drivers.radio.start(15);

	receive(drivers.loop, drivers.node);
	
	drivers.loop.run();
}
