#pragma once

#include <coco/BufferDevice.hpp>
#include <coco/Coroutine.hpp>
#include <coco/enum.hpp>
#include <algorithm>
#include <cstdint>


namespace coco {

/**
 * Interface to an IEEE 802.15.4 radio.
 */
class Ieee802154Radio : public Device {
public:
	using State = Device::State;

	/// filter flags (packets except for PASS_ALL must have a sequence number)
	enum class FilterFlags : uint16_t {
		NONE = 0,

		/**
		 * Pass all packets to receive handler of the context (even when SEQUENCE_NUMBER_SUPPRESSION is on)
		 */
		PASS_ALL = 1,

		/**
		 * Pass beacon packets
		 */
		PASS_TYPE_BEACON = 2,

		/**
		 * Pass packets when destination pan and short address match or are broadcast
		 */
		PASS_DEST_SHORT = 4,

		/**
		 * Pass data packets when destination pan and short address match or are broadcast
		 * (handy for self-powered devices such as EnOcean switches)
		 */
		PASS_TYPE_DATA_DEST_SHORT = 8,

		/**
		 * Pass packets where destination pan and long address match or are broadcast
		 */
		PASS_DEST_LONG = 16,

		/**
		 * Handle ack for packets that pass the filter in radio driver to meet turnaround time.
		 * For received packets, an ACK is sent automatically when the destination address matches.
		 * For sent packets, success is reported after ACK was received. When ACK is not received, resend is attempted
		 * 2 more times
		 */
		HANDLE_ACK = 32,
	};

	enum class SendFlags : uint8_t {
		NONE = 0,

		/// Do not send immediately but wait for a data request from the destination node
		AWAIT_DATA_REQUEST = 1,
	};

	/// Request IDs for remote controlling the radio, e.g. via usb (struct Requet only serves as namespace)
	struct Request {
		//static constexpr uint8_t RESET = 0;

		/// Start radio
		static constexpr uint8_t OPEN = 1;

		/// Stop radio
		static constexpr uint8_t CLOSE = 2;

		/// Configure a virtual node
		static constexpr uint8_t CONFIGURE = 3;
	};

	/**
	 * Maximum payload size without leading length byte and trailing crc
	 */
	static constexpr int MAX_PAYLOAD_SIZE = 125;

	/**
	 * Size of receive header: One byte for link quality indicator (LQI) and 4 bytes for timestamp
	 */
	static constexpr int RECEIVE_HEADER_SIZE = 1 + 4;

	/**
	 * Size of send header: One byte for send flags or length
	 */
	static constexpr int SEND_HEADER_SIZE = 1;

	/**
	 * Maximum header size of radio buffers
	 */
	static constexpr int MAX_HEADER_SIZE = std::max(RECEIVE_HEADER_SIZE, SEND_HEADER_SIZE);

	/**
	 * Size of radio buffer
	 */
	static constexpr int BUFFER_SIZE = MAX_HEADER_SIZE + MAX_PAYLOAD_SIZE;


	Ieee802154Radio(State state) : Device(state) {}
	virtual ~Ieee802154Radio();

	/**
	 * Start the radio. Call close() to stop the radio
	 * @param channel channel 11 - 26, 2400MHz - 2480MHz
	 */
	virtual void open(int channel) = 0;


	class Node : public BufferDevice {
	public:
		Node(State state) : BufferDevice(state) {}
		~Node() override;

		/**
		 * Set configuration pan id, default is 0xffff (broadcast)
		 * @param pan pan id, default is 0xffff (broadcast)
		 * @param longAddress long address
		 * @param shortAddress short address, default is 0xffff (broadcast)
		 * @param filterFlags filter flags, default is none set
		 */
		virtual void configure(uint16_t pan, uint64_t longAddress, uint16_t shortAddress, FilterFlags filterFlags) = 0;
	};

	using Buffer = coco::Buffer;
};
COCO_ENUM(Ieee802154Radio::FilterFlags);
COCO_ENUM(Ieee802154Radio::SendFlags)

} // namespace coco
