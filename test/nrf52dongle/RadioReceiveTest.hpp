#pragma once

#include <coco/platform/Loop_RTC0.hpp>
#include <coco/platform/Ieee802154Radio_RADIO_TIMER0.hpp>


using namespace coco;

// drivers for RadioReceiveTest
struct Drivers {
	Loop_RTC0 loop;

	using Radio = Ieee802154Radio_RADIO_TIMER0;
	Radio radio{loop};
	Radio::Node node{radio};
	Radio::Buffer radioBuffer{node};
};

Drivers drivers;

extern "C" {
void RADIO_IRQHandler() {
	drivers.radio.RADIO_IRQHandler();
}
void TIMER0_IRQHandler() {
	drivers.radio.TIMER0_IRQHandler();
}
}
