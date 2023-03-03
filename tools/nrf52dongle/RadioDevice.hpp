#pragma once

#include <coco/platform/Loop_RTC0.hpp>
#include <coco/platform/UsbDevice_USBD.hpp>
#include <coco/platform/Ieee802154Radio_RADIO_TIMER0_EGU0.hpp>


using namespace coco;

constexpr int NODE_COUNT = 4;

// drivers for RadioDevice
struct Drivers {
	Loop_RTC0 loop;

	// radio
	Ieee802154Radio_RADIO_TIMER0_EGU0 radio{loop};
	using Node = Ieee802154Radio_RADIO_TIMER0_EGU0::Node;
	//Node node{radio};
	Node nodes[NODE_COUNT] = {{radio}, {radio}, {radio}, {radio}};
	using RadioBuffer = Ieee802154Radio_RADIO_TIMER0_EGU0::Buffer;

	// usb
	UsbDevice_USBD device{loop};
	UsbDevice_USBD::ControlBuffer<256> controlBuffer{device};
	using Endpoint = UsbDevice_USBD::BulkEndpoint;
	//Endpoint endpoint{device, 1};
	Endpoint endpoints[NODE_COUNT] = {{device, 1}, {device, 2}, {device, 3}, {device, 4}};
	using UsbBuffer = UsbDevice_USBD::BulkBuffer<Ieee802154Radio::PACKET_LENGTH + 1>;
};
