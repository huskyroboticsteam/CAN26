#pragma once

/**
 * This file contains helper functions to decode packets from the power domain
 */

#include "Power.h"

typedef struct {
    CANDevice_t sender;
    CANDevice_t receiver;
    float voltage;
    float current;
    float soc;
    uint8_t temperature;
} CANPowerPacket_PowerStatus_Decoded_t;

/**
 * Decodes a power status packet into the sender, voltage, current, and consumption
 */
inline static CANPowerPacket_PowerStatus_Decoded_t
CANPowerPacket_PowerStatus_Decode(const CANPacket_t *packet) {
    return (CANPowerPacket_PowerStatus_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .receiver = packet->device,
        .voltage = CANLoadFloat16(packet->contents + 0),
        .current = CANLoadFloat16(packet->contents + 2),
        .soc = CANLoadUNorm8(packet->contents + 4),
        .temperature = (uint8_t)packet->contents[5]
    };
}

typedef struct {
    CANDevice_t sender;
    CANDevice_t receiver;
} CANPowerPacket_GetPowerStatus_Decoded_t;

/**
 * Decodes a power status request packet into the sender and receiver
 */
inline static CANPowerPacket_GetPowerStatus_Decoded_t
CANPowerPacket_GetPowerStatus_Decode(const CANPacket_t *packet) {
    return (CANPowerPacket_GetPowerStatus_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .receiver = packet->device
    };
}