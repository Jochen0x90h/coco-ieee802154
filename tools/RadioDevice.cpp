#include <coco/debug.hpp>
#include <coco/BufferReader.hpp>
#include <coco/BufferWriter.hpp>
#include <RadioDevice.hpp>


// Flash this onto the device to be able to use Ieee802154Radio_usb on your PC to receive and send IEEE 802.15.4 packets.
// Note that the radio needs to be configured and started by the host (using control transfers) before packets are received

using namespace coco;

// device descriptor
static const usb::DeviceDescriptor deviceDescriptor = {
	.bLength = sizeof(usb::DeviceDescriptor),
	.bDescriptorType = usb::DescriptorType::DEVICE,
	.bcdUSB = 0x0200, // USB 2.0
	.bDeviceClass = 0xff, // no class
	.bDeviceSubClass = 0xff,
	.bDeviceProtocol = 0, // 0 = binary, 1 = text
	.bMaxPacketSize0 = 64, // max packet size for endpoint 0
	.idVendor = 0x1915, // Nordic Semiconductor
	.idProduct = 0x1337,
	.bcdDevice = 0x0100, // device version
	.iManufacturer = 0, // index into string table
	.iProduct = 0, // index into string table
	.iSerialNumber = 0, // index into string table
	.bNumConfigurations = 1
};

// configuration descriptor
struct UsbConfiguration {
	struct usb::ConfigurationDescriptor config;
	struct usb::InterfaceDescriptor interface;
	struct usb::EndpointDescriptor endpoints[4 * 2];
} __attribute__((packed));

static const UsbConfiguration configurationDescriptor = {
	.config = {
		.bLength = sizeof(usb::ConfigurationDescriptor),
		.bDescriptorType = usb::DescriptorType::CONFIGURATION,
		.wTotalLength = offsetof(UsbConfiguration, endpoints) + sizeof(usb::EndpointDescriptor) * NODE_COUNT * 2,
		.bNumInterfaces = 1,
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = 0x80, // bus powered
		.bMaxPower = 50 // 100 mA
	},
	.interface = {
		.bLength = sizeof(usb::InterfaceDescriptor),
		.bDescriptorType = usb::DescriptorType::INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = NODE_COUNT * 2,
		.bInterfaceClass = 0xff, // no class
		.bInterfaceSubClass = 0xff,
		.bInterfaceProtocol = 0xff,
		.iInterface = 0
	},
	.endpoints = {
		{
			// endpoint 1 in (device to host, radio receive)
			.bLength = sizeof(usb::EndpointDescriptor),
			.bDescriptorType = usb::DescriptorType::ENDPOINT,
			.bEndpointAddress = 1 | usb::IN,
			.bmAttributes = usb::EndpointType::BULK,
			.wMaxPacketSize = 64,
			.bInterval = 1 // polling interval
		},
		{
			// endpoint 1 out (host to device, radio send)
			.bLength = sizeof(usb::EndpointDescriptor),
			.bDescriptorType = usb::DescriptorType::ENDPOINT,
			.bEndpointAddress = 1 | usb::OUT,
			.bmAttributes = usb::EndpointType::BULK,
			.wMaxPacketSize = 64,
			.bInterval = 1 // polling interval
		},
		{
			// endpoint 2 in (device to host, radio receive)
			.bLength = sizeof(usb::EndpointDescriptor),
			.bDescriptorType = usb::DescriptorType::ENDPOINT,
			.bEndpointAddress = 2 | usb::IN,
			.bmAttributes = usb::EndpointType::BULK,
			.wMaxPacketSize = 64,
			.bInterval = 1 // polling interval
		},
		{
			// endpoint 2 out (host to device, radio send)
			.bLength = sizeof(usb::EndpointDescriptor),
			.bDescriptorType = usb::DescriptorType::ENDPOINT,
			.bEndpointAddress = 2 | usb::OUT,
			.bmAttributes = usb::EndpointType::BULK,
			.wMaxPacketSize = 64,
			.bInterval = 1 // polling interval
		},
		{
			// endpoint 3 in (device to host, radio receive)
			.bLength = sizeof(usb::EndpointDescriptor),
			.bDescriptorType = usb::DescriptorType::ENDPOINT,
			.bEndpointAddress = 3 | usb::IN,
			.bmAttributes = usb::EndpointType::BULK,
			.wMaxPacketSize = 64,
			.bInterval = 1 // polling interval
		},
		{
			// endpoint 3 out (host to device, radio send)
			.bLength = sizeof(usb::EndpointDescriptor),
			.bDescriptorType = usb::DescriptorType::ENDPOINT,
			.bEndpointAddress = 3 | usb::OUT,
			.bmAttributes = usb::EndpointType::BULK,
			.wMaxPacketSize = 64,
			.bInterval = 1 // polling interval
		},
		{
			// endpoint 4 in (device to host, radio receive)
			.bLength = sizeof(usb::EndpointDescriptor),
			.bDescriptorType = usb::DescriptorType::ENDPOINT,
			.bEndpointAddress = 4 | usb::IN,
			.bmAttributes = usb::EndpointType::BULK,
			.wMaxPacketSize = 64,
			.bInterval = 1 // polling interval
		},
		{
			// endpoint 4 out (host to device, radio send)
			.bLength = sizeof(usb::EndpointDescriptor),
			.bDescriptorType = usb::DescriptorType::ENDPOINT,
			.bEndpointAddress = 4 | usb::OUT,
			.bmAttributes = usb::EndpointType::BULK,
			.wMaxPacketSize = 64,
			.bInterval = 1 // polling interval
		}
	}
};

// check if number of virtual raido nodes does not exceed the number of usb endpoints
static_assert(NODE_COUNT * 2 <= std::size(configurationDescriptor.endpoints));


// handle control requests
Coroutine control(Loop &loop, UsbDevice &device, Buffer &buffer, Ieee802154Radio &radio, Drivers::RadioNode *nodes) {
	while (true) {
		usb::Setup setup;

		// wait for a control request (https://www.beyondlogic.org/usbnutshell/usb6.shtml)
		co_await device.request(setup);
		//debug::setRed();

		// handle request
		switch (setup.requestType) {
		case usb::RequestType::STANDARD_DEVICE_IN:
			switch (setup.request) {
			case usb::Request::GET_DESCRIPTOR:
				{
					auto descriptorType = usb::DescriptorType(setup.value >> 8);
					//int descriptorIndex = setup.value & 0xff;
					switch (descriptorType) {
					case usb::DescriptorType::DEVICE:
						co_await UsbDevice::controlIn(buffer, setup, deviceDescriptor);
						break;
					case usb::DescriptorType::CONFIGURATION:
						co_await UsbDevice::controlIn(buffer, setup, configurationDescriptor);
						break;
					//case usb::DescriptorType::STRING:
					default:
						device.stall();
					}
				}
				break;
			default:
				device.stall();
			}
			break;
		case usb::RequestType::VENDOR_DEVICE_OUT:
			switch (Ieee802154Radio::Request(setup.request)) {
			case Ieee802154Radio::Request::START:
				//debug::setBlue();
				device.acknowledge();
				radio.start(setup.value);
				break;
			case Ieee802154Radio::Request::STOP:
				device.acknowledge();
				radio.stop();
				break;
			case Ieee802154Radio::Request::CONFIGURE:
				{
					//debug::setGreen();

					// read uint32 and set as debug color
					co_await buffer.read(setup.length);
					if (setup.index >= 1 && setup.index <= NODE_COUNT && setup.length >= 14 && buffer.size() >= 14) {
						BufferReader r(buffer);
						auto pan = r.u16L();
						auto longAddress = r.u64L();
						auto shortAddress = r.u16L();
						auto filterFlags = r.e16L<Ieee802154Radio::FilterFlags>();
						nodes[setup.index - 1].configure(pan, longAddress, shortAddress, filterFlags);
					}
				}
				break;
			default:
				device.stall();
			}
			break;
		default:
			device.stall();
		}
	}
}

// for each endpoint/node and mac counter a barrier to be able to cancel send requests
Barrier<> barriers[NODE_COUNT][256];

// receive from radio and send to usb host
Coroutine receive(Drivers::RadioNode &node, Drivers::UsbEndpoint &endpoint) {
	Drivers::RadioBuffer radioBuffer(node);
	Drivers::UsbBuffer usbBuffer(endpoint);
	while (true) {
		co_await radioBuffer.untilReady();
		co_await usbBuffer.untilReady();
		while (radioBuffer.ready() && usbBuffer.ready()) {
			// receive from radio
			debug::setGreen(false);
			co_await radioBuffer.read();
			debug::setGreen(true);
			int transferred = radioBuffer.size();

			// check if packet has minimum length of 2 bytes for frame control
			if (transferred >= 2) {
				// send to usb host
				BufferWriter w(usbBuffer);
				int headerSize = radioBuffer.headerSize();
				w.u8(headerSize);
				w.data(radioBuffer.headerData(), headerSize + transferred);
				co_await usbBuffer.write(w); // IN
			}
		}
	}
}

// receive from usb host and send to radio
Coroutine send(Drivers::Radio::Node &node, Drivers::UsbEndpoint &endpoint, int index) {
	Drivers::RadioBuffer radioBuffer(node);
	Drivers::UsbBuffer usbBuffer(endpoint);
	while (true) {
		co_await radioBuffer.untilReady();
		co_await usbBuffer.untilReady();
		while (radioBuffer.ready() && usbBuffer.ready()) {
			// receive from usb host
			co_await usbBuffer.read(); // OUT
			auto data = usbBuffer.data();
			int transferred = usbBuffer.size();

			if (transferred == 1) {
				// cancel by mac counter
				uint8_t macCounter = data[0];
				barriers[index][macCounter].doAll();
			} else {
				int headerSize = data[0];
				int size = transferred - 1 - headerSize;
				if (size >= 2) {
					// get mac counter to identify the packet
					uint8_t macCounter = data[1 + headerSize + 2];

					// set header
					radioBuffer.setHeader(data, headerSize);

					// send over the air
					debug::setRed(true);
					int r = co_await select(radioBuffer.writeData(data + 1 + headerSize, size), barriers[index][macCounter].wait());
					if (r == 1) {
						// send mac counter and number of transferred bytes back to usb host
						usbBuffer[0] = macCounter;
						usbBuffer[1] = radioBuffer.size();
						co_await usbBuffer.write(2); // IN
					}
					debug::setRed(false);
				}
			}
		}
	}
}

int main(void) {
	// handle control transfers from usb host
	control(drivers.loop, drivers.device, drivers.controlBuffer, drivers.radio, drivers.nodes);

	for (int index = 0; index < NODE_COUNT; ++index) {
		for (int i = 0; i < 8; ++i) {
			receive(drivers.nodes[index], drivers.endpoints[index]);
			send(drivers.nodes[index], drivers.endpoints[index], index);
		}
	}


	// configure and start radio
	drivers.nodes[0].configure(0, UINT64_C(0x0000133700001337), 1337,
		Ieee802154Radio::FilterFlags::PASS_DEST_LONG | Ieee802154Radio::FilterFlags::PASS_DEST_SHORT | Ieee802154Radio::FilterFlags::HANDLE_ACK);
	drivers.radio.start(15);


	drivers.loop.run();
}
