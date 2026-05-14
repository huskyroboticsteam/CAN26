#pragma once

/**
 * This file conists of helper functions for packet types from the Peripheral Domain
 */

#include "../CANPacket.h"
#include "../CANCommandIDs.h"

/**
 * Constructs a packet to set the duty cycle of the device with the given pwm ID
 * dutyCycle clamped to the range 0-100
 */
inline static CANPacket_t CANPeripheralPacket_SetPWMDutyCycle(CANDevice_t sender, CANDevice_t device, uint8_t peripheralID, float dutyCycle) {
    CANPacket_t result = {
        .device = device,
        .contentsLength = 5,
        .command = CAN_COMMAND_ID__PWM_DUTY_CYCLE,
        .senderUUID = ((CANDeviceUUID_t)sender.deviceUUID),
        .contents = {peripheralID}
    };
    if (dutyCycle < 0) {
        dutyCycle = 0;
    } else if (dutyCycle > 100) {
        dutyCycle = 100;
    }
    CANStoreFloat32(result.contents + 1, dutyCycle);
    return result;
}
/**
 * 
 * Constructs a packet to run and set the direction of a linear actuator with given ID.
 * To stop, set drive = 0, otherwise, set drive to any positive value for forwards, negative for reverse.
 */
inline static CANPacket_t CANPeripheralPacket_SetLinearActuator(CANDevice_t sender, CANDevice_t device, uint8_t peripheralID, int8_t drive) {
    return (CANPacket_t){
        .device = device,
        .contentsLength = 2,
        .command = CAN_COMMAND_ID__LINEAR_ACTUATOR_CONTROL,
        .senderUUID = ((CANDeviceUUID_t)sender.deviceUUID),
        .contents = {peripheralID, drive}
    };
}

/**
 * Constructs a packet to set the color of the rover LED strips needed for competition
 */
inline static CANPacket_t CANPeripheralPacket_SetRoverLEDColor(CANDevice_t sender, CANDevice_t device, uint8_t red, uint8_t green, uint8_t blue) {
    return (CANPacket_t){
        .device = device,
        .contentsLength = 3,
        .command = CAN_COMMAND_ID__ROVER_LED_COLOR,
        .senderUUID = ((CANDeviceUUID_t)sender.deviceUUID),
        .contents = {red, green, blue}
    };
}

inline static CANPacket_t CANPeripheralPacket_SetBrakes(CANDevice_t sender, CANDevice_t device, uint8_t brake_id, uint8_t state) {
    return (CANPacket_t) {
        .device = device,
        .contentsLength = 2,
        .command = CAN_COMMAND_ID__SET_BRAKE_CONTROL,
        .senderUUID = ((CANDeviceUUID_t)sender.deviceUUID),
        .contents = {brake_id, state}
    };
}

inline static CANPacket_t CANPeripheralPacket_SetRoverLEDRed(CANDevice_t sender, CANDevice_t device) {
    return (CANPacket_t) {
        .device = device,
        .contentsLength = 0,
        .command = CAN_COMMAND_ID__SET_LED_RED,
        .senderUUID = ((CANDeviceUUID_t) sender.deviceUUID),
    };
}

inline static CANPacket_t CANPeripheralPacket_SetRoverLEDBlue(CANDevice_t sender, CANDevice_t device) {
    return (CANPacket_t) {
        .device = device,
        .contentsLength = 0,
        .command = CAN_COMMAND_ID__SET_LED_BLUE,
        .senderUUID = ((CANDeviceUUID_t) sender.deviceUUID),
    };
}

inline static CANPacket_t CANPeripheralPacket_SetRoverFlashLEDGreen(CANDevice_t sender, CANDevice_t device) {
    return (CANPacket_t) {
        .device = device,
        .contentsLength = 0,
        .command = CAN_COMMAND_ID__SET_LED_GREEN_FLASH,
        .senderUUID = ((CANDeviceUUID_t) sender.deviceUUID),
    };
}

inline static CANPacket_t CANPeripheralPacket_SetRoverFlashLEDGreen(CANDevice_t sender, CANDevice_t device) {
    return (CANPacket_t) {
        .device = device,
        .contentsLength = 0,
        .command = CAN_COMMAND_ID__RESET,
        .senderUUID = ((CANDeviceUUID_t) sender.deviceUUID),
    };
}

inline static CANPacket_t CANPeripheralPacket_SetServoAngle(CANDevice_t sender, CANDevice_t device, uint8_t servo_id, uint8_t servo_angle) {
    return (CANPacket_t) {
        .device = device,
        .contentsLength = 2,
        .command = CAN_COMMAND_ID__SERVO_ANGLE,
        .senderUUID = ((CANDeviceUUID_t)sender.deviceUUID),
        .contents = {servo_id, servo_angle}
    };
}
