#pragma once

#include <stdbool.h>

#include "../CANPacket.h"
#include "../CANPacketIDs.h"

// General

inline static CANPacket_t CANMotorPacket_LimitSwitchAlert(CANDeviceUUID_t sender, CANDevice_t device, uint8_t motorId, bool switchStatus) {
    return (CANPacket_t){
        .device = device,
        .contentsLength = 2,
        .command = CAN_PACKET_ID__LIMIT_SWITCH_ALERT,
        .senderUUID = sender,
        .contents = {motorId, switchStatus}
    };
}

// DC Motors

// Stepper Motors

// Positive number of radians is clockwise, negative number is counter-clockwise
inline static CANPacket_t CANMotorPacket_Stepper_DriveRadians(CANDeviceUUID_t sender, CANDevice_t device, float numRadians) {
    CANPacket_t result = {
        .device = device,
        .contentsLength = 4,
        .command = CAN_PACKET_ID__STEPPER_DRIVE_RAD,
        .senderUUID = sender,
    };
    CANStoreFloat32(result.contents + 0, numRadians);
    return result;
}
