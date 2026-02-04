#include <coco/convert.hpp>
#include <coco/debug.hpp>
#include <RadioReceiveTest.hpp>

/*
    This test receives packets and outputs them to the console (native) or sets the debug led color dependent on the packet length.
    ACK is sent when requested, therefore can be used as receiver for RadioSendTest.cpp
*/

// receive packets
Coroutine receive(Loop &loop, Buffer &radioBuffer) {
    while (true) {
        debug::out << "Waiting radio device to become ready...\n";
        co_await radioBuffer.untilReady();

        debug::out  << "Waiting for IEEE 802.15.4 packets...\n";
        while (radioBuffer.ready()) {
            // wait for receive packet
            co_await radioBuffer.read();
            int transferred = radioBuffer.size();

            auto &header = radioBuffer.header<Ieee802154Radio::ReceiveHeader>();
            debug::out << dec(header.timestamp) << ' ' << dec(header.lqi);
            debug::out << " (" << dec(transferred) << ") ";
            for (int i = 0; i < transferred; ++i) {
                if (i != 0)
                    debug::out << ", ";
                debug::out << hex(radioBuffer[i]);
            }
            debug::out << '\n';
#ifndef NATIVE
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
    debug::out << "RadioReceiveTest\n";

    drivers.node.configure(0, UINT64_C(0x0000133700001337), 0x1337,
        Ieee802154Radio::FilterFlags::PASS_DEST_LONG | Ieee802154Radio::FilterFlags::PASS_DEST_SHORT | Ieee802154Radio::FilterFlags::HANDLE_ACK);
//		Ieee802154Radio::FilterFlags::PASS_ALL);

    // start radio
    drivers.radio.open(15);

    receive(drivers.loop, drivers.radioBuffer);

    drivers.loop.run();
}
