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

