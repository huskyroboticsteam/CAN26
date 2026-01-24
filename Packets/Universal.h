#pragma once

/**
 * This file contains helper functions to construct packets from the Universal set of packets
 * These Universal packets must be supported by all devices on the CAN network
 */

#include "../CANPacket.h"
#include "../CANCommandIDs.h"

#include <stdlib.h>
#include <string.h>

/**
 * Returns an emergency stop packet designed to be sent to the given device
 */
inline static CANPacket_t CANUniversalPacket_EStop(CANDevice_t sender, CANDevice_t device) {
    return (CANPacket_t){
        .device = device,
        .priority = CAN_PRIORITY_HIGH,
        .contentsLength = 0,
        .command = CAN_COMMAND_ID__E_STOP,
        .senderUUID = ((CANDeviceUUID_t)sender.deviceUUID)
    };
}

/**
 * Returns a packet that represents a general acknowledgement
 * Should be sent when an acknowledgment was requested but no specific acknowledge packet exists
 */
inline static CANPacket_t CANUniversalPacket_Acknowledge(CANDevice_t sender, CANDevice_t device, bool failure) {
    return (CANPacket_t){
        .device = device,
        .contentsLength = 1,
        .command = CAN_COMMAND_ID__ACKNOWLEDGE,
        .senderUUID = ((CANDeviceUUID_t)sender.deviceUUID),
        .contents = {failure}
    };
}

/**
 * Returns a packet to query the firmware version from the given device
 * Packet is automatically set to acknowledge
 *
 * The firmware version of a device consists of a 16 bit unsigned int alongside a string that is up to 4 bytes long
 * The int should be updated whenever the firmware is updated
 */
inline static CANPacket_t CANUniversalPacket_GetFirmwareVersion(CANDevice_t sender, CANDevice_t device) {
    return (CANPacket_t){
        .device = device,
        .contentsLength = 0,
        .command = CAN_ACK(CAN_COMMAND_ID__VERSION_GET),
        .senderUUID = ((CANDeviceUUID_t)sender.deviceUUID)
    };
}

#define CAN_FIRMWARE_VERSION_LEN 4

/**
 * Returns a packet that encodes the firmware version
 * Should be sent as a response to a GetFirmwareVersion packet
 *
 * Firmware version is a 16 bit unsigned int + a string up to 4 bytes long (eg. odrv v312 for odrive, stpr v4 or something for stepper)
 */
inline static CANPacket_t CANUniversalPacket_FirmwareVersion(CANDevice_t sender, CANDevice_t device, const char *name, uint16_t versionID) {
    size_t nameLength = strlen(name);
    if (nameLength > CAN_FIRMWARE_VERSION_LEN) nameLength = CAN_FIRMWARE_VERSION_LEN;
    CANPacket_t result = {
        .device = device,
        .contentsLength = (uint8_t)(2 + nameLength),
        .command = CAN_COMMAND_ID__VERSION,
        .senderUUID = ((CANDeviceUUID_t)sender.deviceUUID)
    };
    CANStoreUInt16(result.contents + 0, versionID);
    memcpy(result.contents + 2, name, nameLength);
    return result;
}
