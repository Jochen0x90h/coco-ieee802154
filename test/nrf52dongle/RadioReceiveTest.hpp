#pragma once

#include <coco/platform/Loop_RTC0.hpp>
#include <coco/platform/Ieee802154Radio_RADIO_TIMER0_EGU0.hpp>


using namespace coco;

// drivers for SpiTest
struct Drivers {
	Loop_RTC0 loop;
	Ieee802154Radio_RADIO_TIMER0_EGU0 radio{loop};
	Ieee802154Radio_RADIO_TIMER0_EGU0::Node node{radio};
	Ieee802154Radio_RADIO_TIMER0_EGU0::Buffer radioBuffer{node};
};
