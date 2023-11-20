#pragma once

#include <coco/platform/Loop_RTC0.hpp>
#include <coco/platform/UsbDevice_USBD.hpp>
#include <coco/platform/Ieee802154Radio_RADIO_TIMER0.hpp>


using namespace coco;

constexpr int NODE_COUNT = 4;

// drivers for RadioDevice
struct Drivers {
	Loop_RTC0 loop;

	// radio
	using Radio = Ieee802154Radio_RADIO_TIMER0;
	using RadioNode = Radio::Node;
	using RadioBuffer = Radio::Buffer;
	Radio radio{loop};
	RadioNode nodes[NODE_COUNT] = {{radio}, {radio}, {radio}, {radio}};

	// usb
	using UsbDevice = UsbDevice_USBD;
	using UsbEndpoint = UsbDevice::BulkEndpoint;
	using UsbBuffer = UsbDevice::BulkBuffer<1 + Radio::BUFFER_SIZE + 1>;
	UsbDevice device{loop};
	UsbDevice::ControlBuffer<256> controlBuffer{device};
	UsbEndpoint endpoints[NODE_COUNT] = {{device, 1}, {device, 2}, {device, 3}, {device, 4}};
};
