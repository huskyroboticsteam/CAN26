

#if defined(CHIP_TYPE) && CHIP_TYPE == CHIP_TYPE_ESP32
#include "Port.h"
#include "../CANPacket.h"
#include <string.h>
#include <ESP32-TWAI-CAN.hpp>

// Note: Define CAN26 specific universal error codes eventually
#define SUCCESS 0
#define ERROR 1

typedef struct  {
    int tx_pin;
    int rx_pin;
    CANDevice_t device;
} ESP32CANHandle_t;

static twai_filter_config_t createFilterForDevice(const CANDevice_t *device) {
    twai_filter_config_t filterConfig = {};
    // filter 1: uuid match, bits[30:24]. filter 2: UUID==0 (broadcast) [14:8]. Will need software filter for domain matching
    filterConfig.acceptance_code = ((uint32_t)device->deviceUUID << 24);
    // esp32 mask uses inverted logic: 0 = must-match, 1 = dont care
    //   upper 0x80FF -> match bits 30:24 (UUID), ignore priority/domain/RTR/data
    //   lower 0x80FF -> match bits 14:8  (UUID==0), ignore the rest
    filterConfig.acceptance_mask = 0x80FF80FFu;
    filterConfig.single_filter   = false; // See dual filter mode documentation
    return filterConfig;
}

extern "C" uint8_t CANInit(CANHandle_t CANHandle, CANDevice_t *device) {
    if (!CANHandle || !device){
        return ERROR;
    } 
    ESP32CANHandle_t *can = static_cast<ESP32CANHandle_t*>(CANHandle);
    can->device = *device;

    twai_filter_config_t filter_cfg = createFilterForDevice(device);

    // Begin CAN with given pins, 125 kbps baud rate, and tx and rx queue sizes of 99
    if(ESP32Can.begin(ESP32Can.convertSpeed(CAN_BAUD_RATE/1000),  can->tx_pin, can->rx_pin, 99, 99, &filter_cfg)) {
        return SUCCESS;
    } else {
        return ERROR;
    }
}

extern "C" uint8_t CANSend(CANHandle_t CANHandle, const CANPacket_t *packet) {
    if (!CANHandle || !packet) {
        return ERROR;
    }
    CanFrame txFrame = {0};
    txFrame.extd = 0; // standard frame
    txFrame.rtr  = 0;
    txFrame.identifier = CANGetPacketHeader(packet);
    txFrame.data_length_code = CANGetDlc(packet);
    if (txFrame.data_length_code > 8) {
        return ERROR;
    }   
    memcpy(txFrame.data, CANGetDataConst(packet), txFrame.data_length_code);
    if(ESP32Can.writeFrame(&txFrame, 0)) {
        return SUCCESS;
    } else {
        return ERROR;
    }
}

extern "C" int8_t CANPollAndReceive(CANHandle_t CANHandle, CANPacket_t *packet) {
    if (!CANHandle || !packet) {
        return ERROR;
    }
    CanFrame rxFrame;
    if(ESP32Can.readFrame(&rxFrame, 0)) {
        packet->priority = (rxFrame.identifier & (1 << 10)) ? CAN_PRIORITY_LOW : CAN_PRIORITY_HIGH;
        uint16_t deviceBits = rxFrame.identifier & 0x3FF; 
        ESP32CANHandle_t *can = static_cast<ESP32CANHandle_t*>(CANHandle);
        
        if ((deviceBits >> 3) != can->device.deviceUUID) {
            if (!((deviceBits & 0x07) &
                (can->device.peripheralDomain | (can->device.motorDomain << 1) | (can->device.powerDomain << 2)))) {
                return 0;
            }
        }
        packet->device.peripheralDomain = deviceBits & 0x01;
        packet->device.motorDomain = (deviceBits >> 1) & 0x01;
        packet->device.powerDomain = (deviceBits >> 2) & 0x01;
        packet->device.deviceUUID = (deviceBits >> 3) & 0x7F;

        uint8_t dlc = rxFrame.data_length_code;
        if (dlc < 2 || dlc > 8) {
            return -1;
        }
        packet->contentsLength = dlc - 2;
        memcpy(&packet->command, rxFrame.data, 1);
        memcpy(&packet->senderUUID, rxFrame.data + 1, 1);
        memcpy(&packet->contents, rxFrame.data + 2, packet->contentsLength);
        return 1;
    } else {
        return 0; 
    }
}

#endif