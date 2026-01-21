#pragma once

/**
 * This file contains helper functions to decode packets from the peripheral domain
 */

#include "Peripheral.h"

typedef struct {
    CANDevice_t sender;
    uint8_t peripheralID;
    float dutyCycle;
} CANPeripheralPacket_SetPWMDutyCycle_Decoded_t;

/**
 * Decodes a PWM duty cycle packet into its sender, id, and dutycycle value
 */
inline static CANPeripheralPacket_SetPWMDutyCycle_Decoded_t
CANPeripheralPacket_SetPWMDutyCycle_Decode(const CANPacket_t *packet) {
    float dutyCycle = CANLoadFloat32(packet->contents + 1);
    return {
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .peripheralID = packet->contents[0],
        .dutyCycle = dutyCycle
    };
}

typedef struct {
    CANDevice_t sender;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} CANPeripheralPacket_SetRoverLEDColor_Decoded_t;

/**
 * Decodes a rover led color packet into its sender and rgb values
 */
inline static CANPeripheralPacket_SetRoverLEDColor_Decoded_t
CANPeripheralPacket_SetRoverLEDColor_Decode(const CANPacket_t *packet) {
    return {
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .red = packet->contents[0],
        .green = packet->contents[1],
        .blue = packet->contents[2]
    };
}
