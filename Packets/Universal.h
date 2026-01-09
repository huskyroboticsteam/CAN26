#pragma once

#include "../CANPacket.h"
#include "../CANPacketIDs.h"

#include <stdlib.h>
#include <string.h>

inline static CANPacket_t CANUniversalPacket_EStop(CANDeviceUUID_t sender, CANDevice_t device) {
    return (CANPacket_t){
        .device = device,
        .priority = CAN_PRIORITY_HIGH,
        .contentsLength = 0,
        .command = CAN_PACKET_ID__E_STOP,
        .senderUUID = sender
    };
}

inline static CANPacket_t CANUniversalPacket_GetFirmwareVersion(CANDeviceUUID_t sender, CANDevice_t device) {
    return (CANPacket_t){
        .device = device,
        .contentsLength = 0,
        .command = CAN_ACK(CAN_PACKET_ID__VERSION_GET),
        .senderUUID = sender
    };
}

// Firmware version is a 16 bit unsigned int + an up to 4 byte long string (eg. odrv v312 for odrive, stpr v4 or something for stepper)
inline static CANPacket_t CANUniversalPacket_FirmwareVersion(CANDeviceUUID_t sender, CANDevice_t device, const char *name, uint16_t versionID) {
    size_t nameLength = strlen(name);
    if (nameLength > 4) nameLength = 4;
    CANPacket_t result = {
        .device = device,
        .contentsLength = (uint8_t)(2 + nameLength),
        .command = CAN_PACKET_ID__VERSION,
        .senderUUID = sender
    };
    CANStoreUInt16(result.contents + 0, versionID);
    memcpy(result.contents + 2, name, nameLength);
    return result;
}
