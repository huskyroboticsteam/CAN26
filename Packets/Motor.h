#pragma once

/**
 * This file consists of helper functions for packet types from the Motor domain
 * All devices within the motor domain must support all of these packet types
 */

#include <stdbool.h>

#include "../CANPacket.h"
#include "../CANPacketIDs.h"

// General

/**
 * Constructs a packet to send an update from a limit switch to the given device
 */
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

/**
 * Constructs a packet to tell a stepper motor to move a given number of revolutions from the current position
 * Positive number of revolutions is clockwise, negative number is counter-clockwise
 */
inline static CANPacket_t CANMotorPacket_Stepper_DriveRevolutions(CANDeviceUUID_t sender, CANDevice_t device, float numRevolutions) {
    CANPacket_t result = {
        .device = device,
        .contentsLength = 4,
        .command = CAN_PACKET_ID__STEPPER_DRIVE_RAD,
        .senderUUID = sender,
    };
    CANStoreFloat32(result.contents + 0, numRevolutions);
    return result;
}

// BLDC Motors
// All packets currently correspond to Odrive functions

#define BLDC_VOLTAGE_CONTROL  0x00
#define BLDC_TORQUE_CONTROL   0x01
#define BLDC_VELOCITY_CONTROL 0x02
#define BLDC_POSITION_CONTROL 0x03

#define BLDC_INACTIVE_INPUT     0x00
#define BLDC_PASSTHROUGH_INPUT  0x01
#define BLDC_VEL_RAMP_INPUT     0x02
#define BLDC_POS_FILTER_INPUT   0x03
#define BLDC_MIX_CHANNELS_INPUT 0x04
#define BLDC_TRAP_TRAJ_INPUT    0x05
#define BLDC_TORQUE_RAMP_INPUT  0x06
#define BLDC_MIRROR_INPUT       0x07
#define BLDC_TUNING_INPUT       0x08

/**
 * Constructs a packet to set the input mode of the motor
 * controlMode should be one of the BLDC_x_CONTROL macros
 * inputMode should be one of the BLDC_x_INPUT macros
 */
inline static CANPacket_t CANMotorPacket_BLDC_SetInputMode(CANDeviceUUID_t sender, CANDevice_t device, uint8_t controlMode, uint8_t inputMode) {
    return (CANPacket_t){
        .device = device,
        .contentsLength = 2,
        .command = CAN_PACKET_ID__BLDC_INPUT_MODE,
        .senderUUID = sender,
        .contents = {controlMode, inputMode}
    };
}

/**
 * Constructs a packet that sets the destination position and feed forward velocity of the bldc motor
 * The motor should be placed into the BLDC_POSITION_CONTROL mode before this packet is sent
 * position is in units of rev, feedForwardVelocity is in units of rev/s
 * 
 * Feed forward velocity is actually in multiples of 0.001 rev/s, values are clipped into range
 */
inline static CANPacket_t CANMotorPacket_BLDC_SetInputPosition(CANDeviceUUID_t sender, CANDevice_t device, float position, float feedForwardVelocity) {
    uint16_t feedForwardVelocityInt;
    // (the != acts as a NaN check here)
    if (feedForwardVelocity != feedForwardVelocity || feedForwardVelocity < 0) {
        feedForwardVelocityInt = 0;
    } else if (feedForwardVelocity >= 65536.0 * 0.001) {
        feedForwardVelocityInt = 65535;
    } else {
        feedForwardVelocityInt = (uint16_t)(feedForwardVelocity / 0.001);
    }
    CANPacket_t result = {
        .device = device,
        .contentsLength = 6,
        .command = CAN_PACKET_ID__BLDC_INPUT_POSITION,
        .senderUUID = sender
    };
    CANStoreFloat32(result.contents + 0, position);
    CANStoreInt16  (result.contents + 4, feedForwardVelocityInt);
    return result;
}

/**
 * Constructs a packet that sets the destination velocity and feed forward torque of the bldc motor
 * The motor should be placed into the BLDC_VELOCITY_CONTROL mode before this packet is sent
 * velocity in units of rev/s, feedForwardTorque is in units of Nm
 */
inline static CANPacket_t CANMotorPacket_BLDC_SetInputVelocity(CANDeviceUUID_t sender, CANDevice_t device, float velocity, float feedForwardTorque) {
    CANPacket_t result = {
        .device = device,
        .contentsLength = 6,
        .command = CAN_PACKET_ID__BLDC_INPUT_VELOCITY,
        .senderUUID = sender
    };
    CANStoreFloat32(result.contents + 0, velocity);
    CANStoreFloat16(result.contents + 4, feedForwardTorque);
    return result;
}

/**
 * Constructs a packet to directly write to ODrive registers, setting the endpointID to the given value
 */
inline static CANPacket_t CANMotorPacket_BLDC_DirectWrite(CANDeviceUUID_t sender, CANDevice_t device, uint16_t endpointID, uint32_t value) {
    CANPacket_t result = {
        .device = device,
        .contentsLength = 6,
        .command = CAN_PACKET_ID__BLDC_DIRECT_WRITE,
        .senderUUID = sender
    };
    CANStoreUInt16(result.contents + 0, endpointID);
    CANStoreUInt32(result.contents + 2, value);
    return result;
}

/**
 * Constructs a packet to request to directly read from ODrive registers (to read the endpointID register)
 *
 * Implies a reponse in the form of CANMotorPacket_BLDC_DirectReadResult
 */
inline static CANPacket_t CANMotorPacket_BLDC_DirectRead(CANDeviceUUID_t sender, CANDevice_t device, uint16_t endpointID) {
    CANPacket_t result = {
        .device = device,
        .contentsLength = 2,
        .command = CAN_ACK(CAN_PACKET_ID__BLDC_DIRECT_READ),
        .senderUUID = sender
    };
    CANStoreUInt16(result.contents + 0, endpointID);
    return result;
}

/**
 * Constructs a packet to request encoder estimates from the device
 * encoderID is the encoder to read from
 *
 * Implies a response in the form of CANMotorPacket_BLDC_EncoderEstimates
 */
inline static CANPacket_t CANMotorPacket_BLDC_GetEncoderEstimates(CANDeviceUUID_t sender, CANDevice_t device, uint8_t encoderID) {
    return (CANPacket_t){
        .device = device,
        .contentsLength = 1,
        .command = CAN_ACK(CAN_PACKET_ID__BLDC_ENCODER_ESTIMATE_GET),
        .senderUUID = sender,
        .contents = {encoderID}
    };
}

/**
 * Constructs a packet containing encoder estimates from a given encoder
 * Sent as a response to CANMotorPacket_BLDC_GetEncoderEstimates
 *
 * position should be in units of rev
 * velocity should be in units of rev/s
 */
inline static CANPacket_t CANMotorPacket_BLDC_EncoderEstimates(CANDeviceUUID_t sender, CANDevice_t device, float position, float velocity) {
    CANPacket_t result = {
        .device = device,
        .contentsLength = 6,
        .command = CAN_PACKET_ID__BLDC_ENCODER_ESTIMATE,
        .senderUUID = sender
    };
    CANStoreBFloat24(result.contents + 0, position);
    CANStoreBFloat24(result.contents + 3, velocity);
    return result;
}
