#pragma once

#include <coco/platform/Loop_native.hpp>
#include <coco/platform/Ieee802154Radio_usb.hpp>


using namespace coco;

// drivers for SpiTest
struct Drivers {
	Loop_native loop;

	// usb
	UsbHost_native host{loop};
	UsbHost_native::Device device{host, [](const usb::DeviceDescriptor &deviceDescriptor) {
		return deviceDescriptor.idVendor == 0x1915 && deviceDescriptor.idProduct == 0x1337;
	}};
	UsbHost_native::ControlBuffer controlBuffer{device, 32};
	UsbHost_native::BulkEndpoint bulkEndpoint{device, 1};
	UsbHost_native::BulkBuffer bulkBuffer{bulkEndpoint, Ieee802154Radio::PACKET_LENGTH};

	// radio
	using Radio = Ieee802154Radio_usb;
	Radio radio{controlBuffer};
	Radio::Node node{radio, bulkEndpoint};
	Radio::Buffer radioBuffer{node, bulkBuffer};
};
