#pragma once

#include <coco/enum.hpp>

/**
 * Structure of IEEE 802.15.4 header
 *
 * 2 Frame Control Field
 * 1 Sequence Number
 * 2 Destination PAN
 * 2/8 Destination Address
 * 2 Source PAN
 * 1/8 Source Address
 */
namespace ieee802154 {

// ieee 802.15.4 frame control field
enum class FrameControl : uint16_t {
	// frame type
	TYPE_MASK = 7,
	TYPE_BEACON = 0,
	TYPE_DATA = 1,
	TYPE_ACK = 2,
	TYPE_COMMAND = 3,

	SECURITY = 1 << 3,

	FRAME_PENDING = 1 << 4,

	ACKNOWLEDGE_REQUEST = 1 << 5,

	PAN_ID_COMPRESSION = 1 << 6,

	SEQUENCE_NUMBER_SUPPRESSION = 1 << 8,

	DESTINATION_ADDRESSING_MASK = 3 << 10,
	DESTINATION_ADDRESSING_FLAG = 2 << 10,
	DESTINATION_ADDRESSING_LONG_FLAG = 1 << 10,
	DESTINATION_ADDRESSING_NONE = 0,
	DESTINATION_ADDRESSING_SHORT = 2 << 10,
	DESTINATION_ADDRESSING_LONG = 3 << 10,

	SOURCE_ADDRESSING_MASK = 3 << 14,
	SOURCE_ADDRESSING_FLAG = 2 << 14,
	SOURCE_ADDRESSING_LONG_FLAG = 1 << 14,
	SOURCE_ADDRESSING_NONE = 0,
	SOURCE_ADDRESSING_SHORT = 2 << 14,
	SOURCE_ADDRESSING_LONG = 3 << 14,
};
COCO_ENUM(FrameControl)

// ieee 802.15.4 command
enum class Command : uint8_t {
	ASSOCIATION_REQUEST = 1,
	ASSOCIATION_RESPONSE = 2,
	DATA_REQUEST = 4,
	ORPHAN_NOTIFICATION = 6,
	BEACON_REQUEST = 7,
};

// ieee 802.15.4 association request
enum class DeviceInfo : uint8_t {
	ALTERNATE_PAN_COORDINATOR = 1,

	DEVICE_TYPE_MASK = 1 << 1,
	DEVICE_TYPE_RFD = 0, // reduced function device
	DEVICE_TYPE_FFD = 1 << 1, // full function device

	POWER_SOURCE_MASK = 1 << 2,
	POWER_SOURCE_BATTERY = 0,
	POWER_SOURCE_MAINS = 1 << 2,

	RX_ON_WHEN_IDLE = 1 << 3,

	SECURITY_CAPABILITY = 1 << 6,

	ALLOCATE_SHORT_ADDRESS = 1 << 7
};
COCO_ENUM(DeviceInfo)

} // namespace ieee802154
