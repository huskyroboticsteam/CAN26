#pragma once

#include <stdint.h>
#include <stdbool.h>

// uint16 used for alignment
// ordered little endian to match arm
typedef struct {
    uint16_t peripheralDomain : 1;
    uint16_t motorDomain : 1;
    uint16_t powerDomain : 1;

    uint16_t deviceUUID : 7;
} CANDevice_t;

// This may not be necessary if we restrict our target more
#if __STDC_VERSION__ >= 202311L || __cplusplus >= 201103L
    #define SMALL_ENUM typedef enum : uint8_t
#elif defined(__GNUC__)
    #define SMALL_ENUM typedef enum __attribute__((__packed__))
#else
    // Small enums don't exist in this case
    #define SMALL_ENUM typedef enum
#endif

// Low is set as 0 so that it is the default in struct initilization
// The priority is inverted when used
SMALL_ENUM {
    CAN_PRIORITY_LOW = 0,
    CAN_PRIORITY_HIGH
} CANPriority_t;

typedef uint8_t CANCommand_t;
typedef uint8_t CANDeviceUUID_t;

// Example device uuids
#define CAN_UUID_BROADCAST ((CANDeviceUUID_t)0x00)
#define CAN_UUID_JETSON    ((CANDeviceUUID_t)0x01)
#define CAN_UUID_BLDC0     ((CANDeviceUUID_t)0x30)
#define CAN_UUID_SENSOR0   ((CANDeviceUUID_t)0x50)

// Example command ids
#define CAN_COMMAND_ERROR        ((CANCommand_t)0x7F)
#define CAN_COMMAND_ESTOP        ((CANCommand_t)0x00)
#define CAN_COMMAND_PING         ((CANCommand_t)0x01)
#define CAN_COMMAND_READ_VOLTAGE ((CANCommand_t)0x02)

// bitwise ORing a command with ack will request a response
#define CAN_ACK(CMD) (CANCommand_t)(0x80 | (CMD))

typedef struct {
    CANPriority_t priority;
    CANDevice_t device;

    // dlc - 2
    uint8_t contentsLength;

    // note that this part forms a contiguous section of 8 bytes representing the data
    CANCommand_t command;
    CANDeviceUUID_t senderUUID; 
    uint8_t contents[6];
} CANPacket_t;

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

// Example packets
const static CANPacket_t eStopPacket = {
    .priority = CAN_PRIORITY_HIGH,
    .device = fullBroadcast,
    .command = CAN_COMMAND_ESTOP,
    .senderUUID = CAN_UUID_JETSON
};

const static CANPacket_t voltageReadPacket = {
    .device = exampleSensor,
    .contentsLength = 1,
    .command = CAN_ACK(CAN_COMMAND_READ_VOLTAGE),
    .senderUUID = CAN_UUID_JETSON,
    .contents = {4} // maybe we are reading from the 5th cell or something
};

// Functions for retrieving packet components
uint16_t CANGetPacketHeader(CANPacket_t *packet);
uint8_t CANGetDlc(CANPacket_t *packet);
uint8_t *CANGetData(CANPacket_t *packet);

// All packet data is little endian
// Functions for reading packet data
uint32_t CANLoadUInt32(uint8_t *ptr);
int32_t CANLoadInt32(uint8_t *ptr);
uint32_t CANLoadUInt24(uint8_t *ptr);
int32_t CANLoadInt24(uint8_t *ptr);
uint16_t CANLoadUInt16(uint8_t *ptr);
int16_t CANLoadInt16(uint8_t *ptr);

float CANLoadFloat32(uint8_t *ptr);
// BFloat24 and BFloat16 are a truncated variants of IEEE 754 single precision floats
float CANLoadBFloat24(uint8_t *ptr);
float CANLoadBFloat16(uint8_t *ptr);
// Float16 represents a IEEE 754 half precision float
// Float16 Handles subnormals and other edge cases properly in the conversion
float CANLoadFloat16(uint8_t *ptr);

// UNorm refers to a format where integers 0-MAX represent the range 0.0-1.0
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
void CANStoreBFloat24(uint8_t *ptr, float value);
void CANStoreBFloat16(uint8_t *ptr, float value);
void CANStoreFloat16(uint8_t *ptr, float value);

void CANStoreUNorm24(uint8_t *ptr, float value);
void CANStoreUNorm16(uint8_t *ptr, float value);
void CANStoreUNorm8(uint8_t *ptr, float value);
