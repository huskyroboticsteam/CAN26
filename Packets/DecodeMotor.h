#pragma once

/**
 * This file contains helper functions to decode packets from the motor domain
 */

#include "Motor.h"

// General

typedef struct {
    CANDevice_t sender;
    CANDevice_t receiver;
    uint8_t motorID;
    bool switchStatus;
} CANMotorPacket_LimitSwitchAlert_Decoded_t;

/**
 * Decodes a limit switch packet into the sender, motor id, and status of the switch
 */
inline static CANMotorPacket_LimitSwitchAlert_Decoded_t
CANMotorPacket_LimitSwitchAlert_Decode(const CANPacket_t *packet) {
    return (CANMotorPacket_LimitSwitchAlert_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .receiver = packet->device,
        .motorID = packet->contents[0],
        .switchStatus = (bool)packet->contents[1]
    };
}

// Stepper

typedef struct {
    CANDevice_t sender;
    CANDevice_t receiver;
    float numRevolutions;
} CANMotorPacket_Stepper_DriveRevolutions_Decoded_t;

/**
 * Decodes a drive revolutions packet into the sender and number of revolutions
 */
inline static CANMotorPacket_Stepper_DriveRevolutions_Decoded_t
CANMotorPacket_Stepper_DriveRevolutions_Decode(const CANPacket_t *packet) {
    float numRevolutions = CANLoadFloat32(packet->contents + 0);
    return (CANMotorPacket_Stepper_DriveRevolutions_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .receiver = packet->device,
        .numRevolutions = numRevolutions
    };
}

// BLDC (ODrive)

typedef struct {
    CANDevice_t sender;
    CANDevice_t receiver;
    uint8_t controlMode;
    uint8_t inputMode;
} CANMotorPacket_BLDC_SetInputMode_Decoded_t;

/**
 * Decodes an input mode packet into the sender, control mode, and input mode
 */
inline static CANMotorPacket_BLDC_SetInputMode_Decoded_t
CANMotorPacket_BLDC_SetInputMode_Decode(const CANPacket_t *packet) {
    return (CANMotorPacket_BLDC_SetInputMode_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .receiver = packet->device,
        .controlMode = packet->contents[0],
        .inputMode = packet->contents[1]
    };
}

typedef struct {
    CANDevice_t sender;
    CANDevice_t receiver;
    float position;
    float feedForwardVelocity;
} CANMotorPacket_BLDC_SetInputPosition_Decoded_t;

/**
 * Decodes a set input position packet into the sender, position, and feed forward velocity
 */
inline static CANMotorPacket_BLDC_SetInputPosition_Decoded_t
CANMotorPacket_BLDC_SetInputPosition_Decode(const CANPacket_t *packet) {
    float position = CANLoadFloat32(packet->contents + 0);
    float feedForwardVelocity = CANLoadInt16(packet->contents + 4) * 0.001;
    return (CANMotorPacket_BLDC_SetInputPosition_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .receiver = packet->device,
        .position = position,
        .feedForwardVelocity = feedForwardVelocity
    };
}

typedef struct {
    CANDevice_t sender;
    CANDevice_t receiver;
    float velocity;
    float feedForwardTorque;
} CANMotorPacket_BLDC_SetInputVelocity_Decoded_t;

/**
 * Decodes a set input velocity packet into the sender, velocity, and feed forward torque
 */
inline static CANMotorPacket_BLDC_SetInputVelocity_Decoded_t
CANMotorPacket_BLDC_SetInputVelocity_Decode(const CANPacket_t *packet) {
    float velocity = CANLoadFloat32(packet->contents + 0);
    float feedForwardTorque = CANLoadFloat16(packet->contents + 4);
    return (CANMotorPacket_BLDC_SetInputVelocity_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .receiver = packet->device,
        .velocity = velocity,
        .feedForwardTorque = feedForwardTorque
    };
}

typedef struct {
    CANDevice_t sender;
    CANDevice_t receiver;
    uint16_t endpointID;
    uint32_t value;
} CANMotorPacket_BLDC_DirectWrite_Decoded_t;

/**
 * Decodes a direct write packet into the sender, endpointID, and value to write
 */
inline static CANMotorPacket_BLDC_DirectWrite_Decoded_t
CANMotorPacket_BLDC_DirectWrite_Decode(const CANPacket_t *packet) {
    uint16_t endpointID = CANLoadUInt16(packet->contents + 0);
    uint32_t value      = CANLoadUInt32(packet->contents + 2);
    return (CANMotorPacket_BLDC_DirectWrite_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .receiver = packet->device,
        .endpointID = endpointID,
        .value = value
    };
}

typedef struct {
    CANDevice_t sender;
    CANDevice_t receiver;
    uint16_t endpointID;
} CANMotorPacket_BLDC_DirectRead_Decoded_t;

/**
 * Decodes a direct read packet into the sender and endpointID
 * Note that this is the request to read, not the response
 */
inline static CANMotorPacket_BLDC_DirectRead_Decoded_t
CANMotorPacket_BLDC_DirectRead_Decode(const CANPacket_t *packet) {
    uint16_t endpointID = CANLoadUInt16(packet->contents + 0);
    return (CANMotorPacket_BLDC_DirectRead_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .receiver = packet->device,
        .endpointID = endpointID
    };
}

typedef struct {
    CANDevice_t sender;
    CANDevice_t receiver;
    uint16_t endpointID;
    uint32_t value;
} CANMotorPacket_BLDC_DirectReadResult_Decoded_t;

/**
 * Decodes a direct read result packet into the sender, endpointID, and value
 * This is the response to the direct read packet
 */
inline static CANMotorPacket_BLDC_DirectReadResult_Decoded_t
CANMotorPacket_BLDC_DirectReadResult_Decode(const CANPacket_t *packet) {
    uint16_t endpointID = CANLoadUInt16(packet->contents + 0);
    uint32_t value      = CANLoadUInt32(packet->contents + 2);
    return (CANMotorPacket_BLDC_DirectReadResult_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .receiver = packet->device,
        .endpointID = endpointID,
        .value = value
    };
}

typedef struct {
    CANDevice_t sender;
    CANDevice_t receiver;
    uint8_t encoderID;
} CANMotorPacket_BLDC_GetEncoderEstimates_Decoded_t;

/**
 * Decodes an encoder value request packet into the sender and encoderID
 * Note that this the request to read the encoder estimates, not the response
 */
inline static CANMotorPacket_BLDC_GetEncoderEstimates_Decoded_t
CANMotorPacket_BLDC_GetEncoderEstimates_Decode(const CANPacket_t *packet) {
    return (CANMotorPacket_BLDC_GetEncoderEstimates_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .receiver = packet->device,
        .encoderID = packet->contents[0]
    };
}

typedef struct {
    CANDevice_t sender;
    CANDevice_t receiver;
    float position;
    float velocity;
} CANMotorPacket_BLDC_EncoderEstimates_Decoded_t;

/**
 * Decodes an encoder estimates packet into the sender, position, and velocity
 * These are the encoder estimates from the encoder
 */
inline static CANMotorPacket_BLDC_EncoderEstimates_Decoded_t
CANMotorPacket_BLDC_EncoderEstimates_Decode(const CANPacket_t *packet) {
    float position = CANLoadBFloat24(packet->contents + 0);
    float velocity = CANLoadBFloat24(packet->contents + 3);
    return (CANMotorPacket_BLDC_EncoderEstimates_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .receiver = packet->device,
        .position = position,
        .velocity = velocity
    };
}
