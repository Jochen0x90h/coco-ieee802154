#pragma once

#include <coco/platform/Loop_native.hpp>
#include <coco/platform/UsbMonitor_native.hpp>
#include <coco/platform/Ieee802154Radio_usb.hpp>


using namespace coco;

// drivers for RadioSendTest
struct Drivers {
    Loop_native loop;

    using UsbHostDevice = UsbHostDevice_native;
    using Radio = Ieee802154Radio_usb;

    // usb monitor
    UsbMonitor_native monitor{loop};

    // usb
    UsbHostDevice usb{loop};
    UsbHostDevice::ControlBuffer controlBuffer{32, usb};
    UsbHostDevice::Endpoint bulkEndpoint{usb, 1};
    UsbHostDevice::Buffer bulkBuffer{1 + Radio::BUFFER_SIZE + 1, bulkEndpoint};
    UsbHostDevice::Buffer bulkReceiveBuffer{1 + Radio::BUFFER_SIZE + 1, bulkEndpoint};

    // radio
    Radio radio{controlBuffer};
    Radio::Node node{radio, bulkEndpoint};
    Radio::Buffer radioBuffer{node, bulkBuffer};
    Radio::Buffer radioReceiveBuffer{node, bulkReceiveBuffer};

    Drivers() {
        monitor.listenAdd([this](DevicePath path, auto &descriptor, String manufacturer, String product, String serial) {
            debug::out << hex(descriptor.idVendor) << ' ' << hex(descriptor.idProduct) << " (" << path << ")\n";
            if (descriptor.idVendor == 0x1209 && descriptor.idProduct == 0x1337)
                usb.open(path);
        });
    }
};

Drivers drivers;
