

#if defined(CHIP_TYPE) && CHIP_TYPE == CHIP_TYPE_TEENSY_4_X
#include "Port.h"
#include "PortTeensy4x.h"
#include "../CANPacket.h"
#include <string.h>

// Note: Define CAN26 specific universal error codes eventually
#define SUCCESS 0
#define ERROR 1

#define CAN_BAUD_RATE 125000

extern "C" uint8_t CANInit(CANHandle_t CANHandle, CANDevice_t *device) {
    if (!CANHandle || !device){
        return ERROR;
    } 
    auto *can = static_cast<TeensyCANBase*>(CANHandle);
    can->device = *device;
    can->begin();
    can->setBaudRate(CAN_BAUD_RATE);
    can->enableFIFO();
    can->setFIFOFilter(REJECT_ALL);
    can->setMBFilter(REJECT_ALL);
    can->setFIFOFilterRange(0, device->deviceUUID << 3, (device->deviceUUID << 3) + 7, STD);
    can->setFIFOFilterRange(1, 0, 7, STD);  //filter for broadcasts. I think we have to do software filtering for specific domain bits, there's no masking filter available
    return SUCCESS;
}

extern "C" uint8_t CANSend(CANHandle_t CANHandle, const CANPacket_t *packet) {
    if (!CANHandle || !packet) {
        return ERROR;
    }
    auto *can = static_cast<TeensyCANBase*>(CANHandle);
    CAN_message_t msg;
    msg.id  = CANGetPacketHeader(packet);
    msg.len = CANGetDlc(packet);
    memcpy(msg.buf, CANGetDataConst(packet), msg.len);
    return (can->write(msg) == 1) ? SUCCESS : ERROR;
}

int8_t CANPollAndReceive(CANHandle_t CANHandle, CANPacket_t *packet) {
    if (!CANHandle || !packet) {
        return ERROR;
    }
    CAN_message_t msg;
    if (can->readFIFO(msg)) {
        packet->priority = (msg.id & (1 << 10)) ? CAN_PRIORITY_LOW : CAN_PRIORITY_HIGH;
        uint16_t deviceBits = msg.id & 0x3FF; 

        // Software filtering for domain bits: If UUID doesn't match, domain bits must for broadcast. Otherwise reject message.
        if(deviceBits >> 3 != thisDevice.deviceUUID) {
            if(!((deviceBits & 0x07) & (thisDevice.peripheralDomain | (thisDevice.motorDomain << 1) | (thisDevice.powerDomain << 2)))) {
                return 0;
            }
        }         
        packet->device.peripheralDomain = deviceBits & 0x01;
        packet->device.motorDomain = (deviceBits >> 1) & 0x01;
        packet->device.powerDomain = (deviceBits >> 2) & 0x01;
        packet->device.deviceUUID = (deviceBits >> 3) & 0x7F;
        uint8_t dlc = msg.len;
        if (dlc < 2 || dlc > 8) {
            return -1;
        }
        packet->contentsLength = dlc - 2;
        memcpy(&packet->command, msg.buf, 1);
        memcpy(&packet->senderUUID, msg.buf + 1, 1);
        memcpy(&packet->contents, msg.buf + 2, packet->contentsLength);
        return 1;
    } else {
        return 0; 
    }
}

extern "C" int8_t CANPollAndReceive(CANHandle_t CANHandle, CANPacket_t *packet) {
    if (!CANHandle || !packet) {
        return ERROR;
    }
    auto *can = static_cast<TeensyCANBase*>(CANHandle);
    const CANDevice_t &thisDevice = can->device;

    CAN_message_t msg;
    if (!can->readFIFO(msg)) {
        return 0;
    }
    packet->priority = (msg.id & (1 << 10)) ? CAN_PRIORITY_LOW : CAN_PRIORITY_HIGH;
    uint16_t deviceBits = msg.id & 0x3FF;

    // Software filtering for domain bits: If UUID doesn't match, domain bits must for broadcast. Otherwise reject message.
    if ((deviceBits >> 3) != thisDevice.deviceUUID) {
        if (!((deviceBits & 0x07) &
              (thisDevice.peripheralDomain | (thisDevice.motorDomain << 1) | (thisDevice.powerDomain << 2)))) {
            return 0;
        }
    }
    packet->device.peripheralDomain = deviceBits & 0x01;
    packet->device.motorDomain      = (deviceBits >> 1) & 0x01;
    packet->device.powerDomain      = (deviceBits >> 2) & 0x01;
    packet->device.deviceUUID       = (deviceBits >> 3) & 0x7F;

    uint8_t dlc = msg.len;
    if (dlc < 2 || dlc > 8) return -1;
    packet->contentsLength = dlc - 2;
    memcpy(&packet->command,    msg.buf,     1);
    memcpy(&packet->senderUUID, msg.buf + 1, 1);
    memcpy(&packet->contents,   msg.buf + 2, packet->contentsLength);
    return 1;
}

#endif