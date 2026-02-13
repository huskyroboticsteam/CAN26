/**This module interfaces with the FDCAN driver functions from the stm32g4xx_hal_fdcan.c
 * file provided by stm. I assume it would workfor any other stm32 family chip that also
 * feature FDCAN, but this has not been tested. 
 * Todos: Define real error codes separate from HAL Error code definitions (athough they can overlap)
 *        Check for error codes returned by any HAL function and add handling.
 *        Determine BAUD rate. Explore other parameters like auto re-transmission, error checking, etc.
 *        
 */

#if defined(CHIP_TYPE) && CHIP_TYPE == CHIP_TYPE_STM32_G4XX
#include "Port.h"
#include "../CANPacket.h"
#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <string.h>

// Bit positions of specific address portions for filter detection
#define PRIORITY_POS       10
#define UUID_POS           3
#define GROUP_MASK_POS     0

// Settings for standard CAN operation (no FD mode)
static const FDCAN_TxHeaderTypeDef txHeaderCANStandard = {
    .IdType = FDCAN_STANDARD_ID,
    .TxFrameType = FDCAN_DATA_FRAME,
    .FDFormat = FDCAN_CLASSIC_CAN,
    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
    .BitRateSwitch = FDCAN_BRS_OFF,
    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    .MessageMarker = 0
};


uint8_t CANInit(CANHandle_t CANHandle, CANDevice_t *CANDevice) {
    if (!CANHandle || !CANDevice) {
        return HAL_ERROR;
    }
    
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)CANHandle;
    FDCAN_FilterTypeDef filterConfig;

    // Filter 0: Filters for only messages matching this devices UUID
    filterConfig.IdType = FDCAN_STANDARD_ID;
    filterConfig.FilterIndex = 0;
    filterConfig.FilterType = FDCAN_FILTER_MASK;
    filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filterConfig.FilterID1 = (CANDevice->deviceUUID << UUID_POS);
    filterConfig.FilterID2 = (0x7F << UUID_POS);

    HAL_FDCAN_ConfigFilter(hfdcan, &filterConfig);

    // Filters 1-3: Filters for group broadcasts matching this device's declared domains
    if (CANDevice->peripheralDomain) { 
        filterConfig.FilterIndex = 1;
        filterConfig.FilterID1 = (0x00 << UUID_POS) | 0x01; // UUID=0, periphereral domain set
        filterConfig.FilterID2 = (0x7F << UUID_POS) | 0x01; // Mask for uuid and last domain bit
        HAL_FDCAN_ConfigFilter(hfdcan, &filterConfig);
    }
    
    if (CANDevice->motorDomain) {
        filterConfig.FilterIndex = 2;
        filterConfig.FilterID1 = (0x00 << UUID_POS) | 0x02;
        filterConfig.FilterID2 = (0x7F << UUID_POS) | 0x02;
        HAL_FDCAN_ConfigFilter(hfdcan, &filterConfig);
    }
    
    if (CANDevice->powerDomain) {
        filterConfig.FilterIndex = 3;
        filterConfig.FilterID1 = (0x00 << UUID_POS) | 0x04;
        filterConfig.FilterID2 = (0x7F << UUID_POS) | 0x04;
        HAL_FDCAN_ConfigFilter(hfdcan, &filterConfig);
    }


    return (uint8_t)HAL_FDCAN_Start(hfdcan); // Needed to activate CAN node, must be done after configuration of filters and optional features. 

}


uint8_t CANSend(CANHandle_t CANHandle, const CANPacket_t *CANPacket) {
    if (!CANHandle || !CANPacket) {
        return HAL_ERROR;
    }
    
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)CANHandle;
    FDCAN_TxHeaderTypeDef messageHeader = txHeaderCANStandard;
    messageHeader.Identifier = CANGetPacketHeader(CANPacket);
    messageHeader.DataLength = CANGetDlc(CANPacket);

    return (uint8_t)HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &messageHeader, CANGetDataConst(CANPacket));
}



int8_t CANPollAndReceive(CANHandle_t CANHandle, CANPacket_t *RxPacket) {
    if (!CANHandle || !RxPacket) {
        return HAL_ERROR;
    }
    
    FDCAN_HandleTypeDef *hfdcan = (FDCAN_HandleTypeDef *)CANHandle;
    if(!HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0)) {
        return 0;
    } else { // messages present in FIFO
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t pRxData[8];
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, pRxData);
        RxPacket->priority = (RxHeader.Identifier & (1 << 10)) ? CAN_PRIORITY_LOW : CAN_PRIORITY_HIGH;

        uint16_t deviceBits = RxHeader.Identifier & 0x3FF; 
        RxPacket->device.peripheralDomain = deviceBits & 0x01;
        RxPacket->device.motorDomain = (deviceBits >> 1) & 0x01;
        RxPacket->device.powerDomain = (deviceBits >> 2) & 0x01;
        RxPacket->device.deviceUUID = (deviceBits >> 3) & 0x7F;
        uint8_t dlc = RxHeader.DataLength;
        if (dlc < 2 || dlc > 8) {
            return HAL_ERROR;
        }
        RxPacket->contentsLength = dlc - 2;
        memcpy(&RxPacket->command, pRxData, 1);
        memcpy(&RxPacket->senderUUID, pRxData + 1, 1);
        memcpy(&RxPacket->contents, pRxData + 2, RxPacket->contentsLength);
        return 1;
    }
}

#endif // defined(CHIP_TYPE) &&CHIP_TYPE == CHIPT_TYPE_STM32_G4XX
