#pragma once

#include <coco/platform/Loop_native.hpp>
#include <coco/platform/Ieee802154Radio_usb.hpp>


using namespace coco;

// drivers for SpiTest
struct Drivers {
	Loop_native loop;
	UsbHost_native host{loop};
	UsbHost_native::Device device{host, [](const usb::DeviceDescriptor &deviceDescriptor) {
		return deviceDescriptor.idVendor == 0x1915 && deviceDescriptor.idProduct == 0x1337;
	}};
	Ieee802154Radio_usb radio{device};
	Ieee802154Radio_usb::Node node{radio};
	Ieee802154Radio_usb::Buffer receiveBuffer{node};
};
