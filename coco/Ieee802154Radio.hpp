#pragma once

#include <coco/BufferDevice.hpp>
#include <coco/Coroutine.hpp>
#include <coco/enum.hpp>
#include <algorithm>
#include <cstdint>


namespace coco {

/**
	Interface to an IEEE 802.15.4 radio.
*/
class Ieee802154Radio {
public:
	using State = Device::State;

	/// filter flags (packets except for PASS_ALL must have a sequence number)
	enum class FilterFlags : uint16_t {
		NONE = 0,

		/**
			Pass all packets to receive handler of the context (even when SEQUENCE_NUMBER_SUPPRESSION is on)
		*/
		PASS_ALL = 1,

		/**
			Pass beacon packets
		*/
		PASS_TYPE_BEACON = 2,

		/**
			Pass packets when destination pan and short address match or are broadcast
		*/
		PASS_DEST_SHORT = 4,

		/**
			Pass data packets when destination pan and short address match or are broadcast
			(handy for self-powered devices such as EnOcean switches)
		*/
		PASS_TYPE_DATA_DEST_SHORT = 8,

		/**
			Pass packets where destination pan and long address match or are broadcast
		*/
		PASS_DEST_LONG = 16,

		/**
			Handle ack for packets that pass the filter in radio driver to meet turnaround time.
			For received packets, an ACK is sent automatically when the destination address matches.
			For sent packets, success is reported after ACK was received. When ACK is not received, resend is attempted
			2 more times
		*/
		HANDLE_ACK = 32,
	};

	enum class SendFlags : uint8_t {
		NONE = 0,

		// do not send immediately but wait for a data request from the destination node
		AWAIT_DATA_REQUEST = 1,
	};

	/// enum for remote controlling the radio, e.g. via usb
	enum class Request : uint8_t {
		// global
		RESET = 0,
		START = 1,
		STOP = 2,
		CONFIGURE = 3
	};

	/**
		Maximum payload length without leading length byte and trailing crc
	*/
	static constexpr int MAX_PAYLOAD_SIZE = 125;

	/**
		Receive header length: One byte for link quality indicator (LQI) and 4 bytes for timestamp
	*/
	//static constexpr int RECEIVE_EXTRA_LENGTH = 1 + 4;
	static constexpr int RECEIVE_HEADER_SIZE = 1 + 4;

	/**
		Send hader length: One byte for send flags or length
	*/
	//static constexpr int SEND_EXTRA_LENGTH = 1;
	static constexpr int SEND_HEADER_SIZE = 1;

	/**
		Length of header for radio buffers
	*/
	static constexpr int MAX_HEADER_SIZE = std::max(RECEIVE_HEADER_SIZE, SEND_HEADER_SIZE);

	/**
		Length of a packet including extra data
	*/
	//static constexpr int PACKET_LENGTH = MAX_PAYLOAD_LENGTH + std::max(RECEIVE_EXTRA_LENGTH, SEND_EXTRA_LENGTH);


	virtual ~Ieee802154Radio();

	/**
		Start the radio in receiving mode but with no baseband decoding. For this, call enableReceive(true)
		@param channel 10 - 26, 2400MHz - 2480MHz
	*/
	virtual void start(int channel) = 0;

	/**
		Stop the radio. Need to call start() again
	*/
	virtual void stop() = 0;


	class Node : public BufferDevice {
	public:
		~Node() override;

		/**
			Set configuration pan id, default is 0xffff (broadcast)
			@param pan pan id, default is 0xffff (broadcast)
			@param longAddress long address
			@param shortAddress short address, default is 0xffff (broadcast)
			@param filterFlags filter flags, default is none set
		*/
		virtual void configure(uint16_t pan, uint64_t longAddress, uint16_t shortAddress, FilterFlags filterFlags) = 0;
	};

	using Buffer = coco::Buffer;
};
COCO_ENUM(Ieee802154Radio::FilterFlags);
COCO_ENUM(Ieee802154Radio::SendFlags)

} // namespace coco
