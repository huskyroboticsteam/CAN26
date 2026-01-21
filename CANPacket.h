#pragma once

#include "CANHelpers.h"

#include <stdint.h>
#include <stdbool.h>

/**
 * Uniquely represents a device on the CAN network
 *
 * uint16 is used for alignment purposes
 * ordered little endian to match arm
 *
 * Note that the domains don't necessarily have to match the device for the packet to be received,
 * but specifying the domain is necessary for broadcasts
 */
typedef struct {
    uint16_t peripheralDomain : 1;
    uint16_t motorDomain : 1;
    uint16_t powerDomain : 1;

    uint16_t deviceUUID : 7;
} CANDevice_t;

/**
 * Represents the priority of a CAN packet
 * Note that in reality LOW corresponds to 1 on the physical layer
 * However, struct initilization defaults to zero, and LOW should be the default
 * 
 * The priority gets inverted when used
 */
SMALL_ENUM {
    CAN_PRIORITY_LOW = 0,
    CAN_PRIORITY_HIGH
} CANPriority_t;

/**
 * Used to represent the command id of the packet
 */
typedef uint8_t CANCommand_t;

/**
 * Used to uniquely identify a device on the CAN network
 * Note that this UUID is the same as the one in CANDevice_t except that one has to be uint16 for alignment
 */
typedef uint8_t CANDeviceUUID_t;

// Example device uuids
#define CAN_UUID_BROADCAST ((CANDeviceUUID_t)0x00)
#define CAN_UUID_JETSON    ((CANDeviceUUID_t)0x01)
#define CAN_UUID_BLDC0     ((CANDeviceUUID_t)0x30)
#define CAN_UUID_SENSOR0   ((CANDeviceUUID_t)0x50)

/**
 * Used to turn a command id into its corresponding command with acknowledgement request
 * Bitwise ORing a command with 0x80 (ack) will request a response
 */
#define CAN_ACK(CMD) (CANCommand_t)(0x80 | (CMD))

/**
 * Represents a packet to be sent on the CAN network
 * Note that the content length does not count the command and senderUUID
 * Everything is little endian where applicable
 */
typedef struct {
    CANDevice_t device;
    // placed after device to remove padding
    CANPriority_t priority;

    // dlc - 2
    uint8_t contentsLength;

    // note that this part forms a contiguous section of 8 bytes representing the data
    CANCommand_t command;
    CANDeviceUUID_t senderUUID; 
    uint8_t contents[6];
} CANPacket_t;

// Note about struct initializers: C allows any order, C++ requires them to be in order
// Example devices
const static CANDevice_t fullBroadcast = {
    .peripheralDomain = true,
    .motorDomain = true,
    .powerDomain = true,
    .deviceUUID = CAN_UUID_BROADCAST
};

const static CANDevice_t jetson = {
    .deviceUUID = CAN_UUID_JETSON
};

const static CANDevice_t exampleBldc = {
    .motorDomain = true,
    .deviceUUID = CAN_UUID_BLDC0
};

const static CANDevice_t exampleSensor = {
    .peripheralDomain = true,
    .deviceUUID = CAN_UUID_SENSOR0
};

// Functions for retrieving packet components

/**
 * Returns the 11 bit structure representing the CAN packet header
 * Includes priority and device
 */
uint16_t CANGetPacketHeader(CANPacket_t *packet);

/**
 * Returns the true data length of the CAN packet (including command id and sender id)
 */
uint8_t CANGetDlc(CANPacket_t *packet);

/**
 * Returns a pointer to the 8 byte data section of the CAN packet (including command id and sender id)
 */
uint8_t *CANGetData(CANPacket_t *packet);

// Functions for reading packet data
/* Overview of available formats
 * name     - size - type
 * UInt32   - 32b  - unsigned integer
 * Int32    - 32b  - signed integer
 * Uint24   - 24b  - unsigned integer
 * Int24    - 24b  - signed integer
 * UInt16   - 16b  - unsigned integer
 * Int16    - 16b  - signed integer
 * Float32  - 32b  - IEEE-754 float
 * Float16  - 16b  - IEEE-754 float (half precision)
 * BFloat24 - 24b  - non standard float (truncated Float32)
 * BFloat16 - 16b  - brain float (truncated Float32)
 * UNorm24  - 24b  - unsigned normalized value (range [0, 1])
 * UNorm16  - 16b  - unsigned normalized value (range [0, 1])
 * UNorm8   - 8b   - unsigned normalized value (range [0, 1])
 *
 * All values are rounded to nearest (ties away from 0) when applicable
 * All data is little endian
 */

uint32_t CANLoadUInt32(uint8_t *ptr);
int32_t CANLoadInt32(uint8_t *ptr);

uint32_t CANLoadUInt24(uint8_t *ptr);
int32_t CANLoadInt24(uint8_t *ptr);

uint16_t CANLoadUInt16(uint8_t *ptr);
int16_t CANLoadInt16(uint8_t *ptr);

float CANLoadFloat32(uint8_t *ptr);
float CANLoadFloat16(uint8_t *ptr);

float CANLoadBFloat24(uint8_t *ptr);
float CANLoadBFloat16(uint8_t *ptr);

float CANLoadUNorm24(uint8_t *ptr);
float CANLoadUNorm16(uint8_t *ptr);
float CANLoadUNorm8(uint8_t *ptr);

// Functions for writing packet data

void CANStoreUInt32(uint8_t *ptr, uint32_t value);
void CANStoreInt32(uint8_t *ptr, int32_t value);

void CANStoreUInt24(uint8_t *ptr, int32_t value);
void CANStoreInt24(uint8_t *ptr, int32_t value);

void CANStoreUInt16(uint8_t *ptr, uint16_t value);
void CANStoreInt16(uint8_t *ptr, int16_t value);

void CANStoreFloat32(uint8_t *ptr, float value);
void CANStoreFloat16(uint8_t *ptr, float value);

void CANStoreBFloat24(uint8_t *ptr, float value);
void CANStoreBFloat16(uint8_t *ptr, float value);

void CANStoreUNorm24(uint8_t *ptr, float value);
void CANStoreUNorm16(uint8_t *ptr, float value);
void CANStoreUNorm8(uint8_t *ptr, float value);
