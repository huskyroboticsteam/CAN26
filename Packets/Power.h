#pragma once

/**
 * This file consists of helper functions for packet types from the Power domain
 */

#include "../CANPacket.h"
#include "../CANCommandIDs.h"

/**
 * Constructs a packet to report the current power status of the device with the following:
 * voltage, current, temperature (fahrenheit), and state of charge (percentage).
 */
inline static CANPacket_t CANPowerPacket_PowerStatus(CANDevice_t sender, CANDevice_t device, float voltage, float current, float soc, uint8_t temperature) {
    CANPacket_t result = {
        .device = device,
        .contentsLength = 8,
        .command = CAN_COMMAND_ID__POWER_STATUS,
        .senderUUID = ((CANDeviceUUID_t)sender.deviceUUID)
    };
    CANStoreFloat16(result.contents + 0, voltage);
    CANStoreFloat16(result.contents + 2, current);
    CANStoreUNorm8(result.contents + 4, soc);
    result.contents[5] = (uint8_t)temperature;
    return result;
}

/**
 * Constructs a packet to request power status
 */
inline static CANPacket_t CANPowerPacket_GetPowerStatus(CANDevice_t sender, CANDevice_t device) {
    return (CANPacket_t){
        .device = device,
        .contentsLength = 0,
        .command = CAN_COMMAND_ID__POWER_STATUS_GET,
        .senderUUID = ((CANDeviceUUID_t)sender.deviceUUID)
    };
}