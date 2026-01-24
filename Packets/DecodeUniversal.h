#pragma once

/**
 * This file contains routines to decode the Universal packets
 * The results of decoding are placed into a struct
 * This allows one to do something like:
 *   CANUniversalPacket_FirmwareVersion_Decode(&packet).versionID
 */

#include "Universal.h"

typedef struct {
    CANDevice_t sender;
} CANUniversalPacket_EStop_Decoded_t;

/**
 * Decodes an EStop packet into its sender
 */
inline static CANUniversalPacket_EStop_Decoded_t
CANUniversalPacket_EStop_Decode(const CANPacket_t *packet) {
    return (CANUniversalPacket_EStop_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID}
    };
}

typedef struct {
    CANDevice_t sender;
    bool failure;
} CANUniversalPacket_Acknowledge_Decoded_t;

/**
 * Decodes a general acknowledge packet into its sender and whether the request failed
 */
inline static CANUniversalPacket_Acknowledge_Decoded_t
CANUniversalPacket_Acknowledge_Decode(const CANPacket_t *packet) {
    return (CANUniversalPacket_Acknowledge_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .failure = (bool)packet->contents[0]
    };
}

typedef struct {
    CANDevice_t sender;
} CANUniversalPacket_GetFirmwareVersion_Decoded_t;

/**
 * Decodes a get firmware version packet into its sender
 */
inline static CANUniversalPacket_GetFirmwareVersion_Decoded_t
CANUniversalPacket_GetFirmwareVersion_Decode(const CANPacket_t *packet) {
    return (CANUniversalPacket_GetFirmwareVersion_Decoded_t){
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID}
    };
}

typedef struct {
    CANDevice_t sender;
    uint16_t versionID;
    char name[CAN_FIRMWARE_VERSION_LEN];
} CANUniversalPacket_FirmwareVersion_Decoded_t;

/**
 * Decodes a firmware version packet into the version name and id
 */
inline static CANUniversalPacket_FirmwareVersion_Decoded_t
CANUniversalPacket_FirmwareVersion_Decode(const CANPacket_t *packet) {
    uint16_t versionID = CANLoadUInt16(packet->contents + 0);
    CANUniversalPacket_FirmwareVersion_Decoded_t result = {
        .sender = (CANDevice_t){.deviceUUID = packet->senderUUID},
        .versionID = versionID
    };
    for (int i = 0; i < CAN_FIRMWARE_VERSION_LEN; ++i) {
        result.name[i] = packet->contents[i + 2];
    }
    return result;
}
