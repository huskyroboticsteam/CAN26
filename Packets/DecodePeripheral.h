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
    return (CANPeripheralPacket_SetPWMDutyCycle_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .peripheralID = packet->contents[0],
        .dutyCycle = dutyCycle
    };
}

typedef struct {
    CANDevice_t sender;
    uint8_t peripheralID;
    int8_t drive;
} CANPeripheralPacket_SetLinearActuator_Decoded_t;

/*
Decodes a linear actuator control packet into its sender, id, and drive value.
*/
inline static CANPeripheralPacket_SetLinearActuator_Decoded_t
CANPeripheralPacket_SetLinearActuator_Decode(const CANPacket_t *packet) {
    return (CANPeripheralPacket_SetLinearActuator_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .peripheralID = packet->contents[0],
        .drive = (int8_t)packet->contents[1]
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
    return (CANPeripheralPacket_SetRoverLEDColor_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .red = packet->contents[0],
        .green = packet->contents[1],
        .blue = packet->contents[2]
    };
}

typedef struct {
    CANDevice_t sender;
    uint8_t brake_id;
    uint8_t state;
} CANPeripheralPacket_SetBrakeControl_Decoded_t;

inline static CANPeripheralPacket_SetBrakeControl_Decoded_t
CANPeripheralPacket_SetBrakes_Decode(const CANPacket_t *packet) {
    return (CANPeripheralPacket_SetBrakeControl_Decoded_t) {
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .brake_id = packet->contents[0],
        .state = packet->contents[1]
    };
}

typedef struct {
    CANDevice_t sender;
} CANPeripheralPacket_SetRoverLEDRed_Decoded_t;

inline static CANPeripheralPacket_SetRoverLEDRed_Decoded_t
CANPeripheralPacket_SetRoverLEDRed_Decode(const CANPacket_t *packet) {
    return (CANPeripheralPacket_SetRoverLEDRed_Decoded_t) {
        .sender = (CANDevice_t) {.deviceUUID = packet->senderUUID}
    };
}

typedef struct {
    CANDevice_t sender;
} CANPeripheralPacket_SetRoverLEDBlue_Decoded_t;

inline static CANPeripheralPacket_SetRoverLEDBlue_Decoded_t
CANPeripheralPacket_SetRoverLEDBlue_Decode(const CANPacket_t *packet) {
    return (CANPeripheralPacket_SetRoverLEDBlue_Decoded_t) {
        .sender = (CANDevice_t) {.deviceUUID = packet->senderUUID}
    };
}

typedef struct {
    CANDevice_t sender;
} CANPeripheralPacket_SetRoverLEDGreenFlash_Decoded_t;

inline static CANPeripheralPacket_SetRoverLEDGreenFlash_Decoded_t
CANPeripheralPacket_SetRoverLEDGreenFlash_Decode(const CANPacket_t *packet) {
    return (CANPeripheralPacket_SetRoverLEDGreenFlash_Decoded_t) {
        .sender = (CANDevice_t) {.deviceUUID = packet->senderUUID}
    };
}

typedef struct {
    CANDevice_t sender;
    uint8_t servo_id;
    uint8_t servo_angle;
} CANPeripheralPacket_SetServoAngleDecoded_t;

inline static CANPeripheralPacket_SetServoAngleDecoded_t
CANPeripheralPacket_SetServoAngle_Decode(const CANPacket_t *packet) {
    return (CANPeripheralPacket_SetServoAngleDecoded_t) {
        .sender = (CANDevice_t) {.deviceUUID = packet->senderUUID},
        .servo_id = packet->contents[0],
        .servo_angle = packet->contents[1]
    };
}

typedef struct {
    CANDevice_t sender;
} CANPeripheralPacket_SetReset_Decoded_t;

inline static CANPeripheralPacket_SetReset_Decoded_t
CANPeripheralPacket_SetReset_Decode(const CANPacket_t *packet) {
    return (CANPeripheralPacket_SetReset_Decoded_t) {
        .sender = (CANDevice_t) {.deviceUUID = packet->senderUUID}
    };
}