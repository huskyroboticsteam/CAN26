#pragma once

/** This header file declares the generic functions that this CAN implementation provides for
 * basic setup and RX/TX. The only currently supported chipset is the STM32G4 family.
 * 
 */

#include "../CANPacket.h"

#define CHIP_TYPE_STM32_G4XX         0x02

// Generic pointer for CAN Handles, should cast to pointer of whatever handle given chipset uses for CAN
typedef void* CAN_Handle_t;


/** 
 * Initialize the CAN for this device with filters and the receive queue.
 * @param hfdcan Pointer for STM32's FDCAN handle
 * @param CANDevice This device's UUID and domain bits.
 * @return 0 for succesful setup, error codes otherwise.
 */
uint8_t CANInit(CAN_Handle_t CANHandle, CANDevice_t *device);

/**
 *  Send a CAN Packet for hardware transmission.
 *  @param CANHandle Pointer for chip specific CAN Handle structure
 *  @param RxPacket Pointer to CAN26 packet struct already filled with data.
 *  @return 0 if no error encountered, error codes otherwise.
 */
uint8_t CANSend(CAN_Handle_t CANHandle, const CANPacket_t *packet);

/**
 *  Check FIFO for received CAN Packets and parse first if present.
 *  @param CANHandle Pointer for chip specific CAN Handle structure
 *  @param RxPacket Pointer to CAN26 packet struct to fill with received data
 *  @return 1 if message was present, 0 if no messages in FIFO, negative if error encountered.
 */
int8_t CANPollAndReceive(CAN_Handle_t CANHandle, CANPacket_t *packet);
