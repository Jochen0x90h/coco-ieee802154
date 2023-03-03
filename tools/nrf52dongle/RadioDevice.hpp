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
	using Node = Ieee802154Radio_RADIO_TIMER0_EGU0::Node;
	using RadioBuffer = Ieee802154Radio_RADIO_TIMER0_EGU0::Buffer;
	Ieee802154Radio_RADIO_TIMER0_EGU0 radio{loop};
	Node nodes[NODE_COUNT] = {{radio}, {radio}, {radio}, {radio}};

	// usb
	using Endpoint = UsbDevice_USBD::BulkEndpoint;
	using UsbBuffer = UsbDevice_USBD::BulkBuffer<Ieee802154Radio::PACKET_LENGTH + 1>;
	UsbDevice_USBD device{loop};
	UsbDevice_USBD::ControlBuffer<256> controlBuffer{device};
	Endpoint endpoints[NODE_COUNT] = {{device, 1}, {device, 2}, {device, 3}, {device, 4}};
};
