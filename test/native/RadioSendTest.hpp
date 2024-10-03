#pragma once

#include <coco/platform/Loop_native.hpp>
#include <coco/platform/Ieee802154Radio_usb.hpp>


using namespace coco;

// drivers for RadioSendTest
struct Drivers {
	Loop_native loop;

	using UsbHost = UsbHost_native;
	using Radio = Ieee802154Radio_usb;

	// usb
	UsbHost host{loop};
	UsbHost::Device device{host, [](const usb::DeviceDescriptor &deviceDescriptor) {
		return deviceDescriptor.idVendor == 0x1915 && deviceDescriptor.idProduct == 0x1337;
	}};
	UsbHost::ControlBuffer controlBuffer{32, device};
	UsbHost::Endpoint bulkEndpoint{device, 1};
	UsbHost::Buffer bulkBuffer{1 + Radio::BUFFER_SIZE + 1, bulkEndpoint};
	UsbHost::Buffer bulkReceiveBuffer{1 + Radio::BUFFER_SIZE + 1, bulkEndpoint};

	// radio
	Radio radio{controlBuffer};
	Radio::Node node{radio, bulkEndpoint};
	Radio::Buffer radioBuffer{node, bulkBuffer};
	Radio::Buffer radioReceiveBuffer{node, bulkReceiveBuffer};
};

Drivers drivers;
