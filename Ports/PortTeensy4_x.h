#pragma once
#include <FlexCAN_T4.h>

extern "C" {
#include "Port.h"
#include "../CANPacket.h"  
}


// Base class was needed since each FlexCAN_T4 instance is a different type
class TeensyCANBase {
public:
    virtual ~TeensyCANBase() = default;
    virtual void begin() = 0;
    virtual void setBaudRate(uint32_t baud) = 0;
    virtual void enableFIFO() = 0;
    virtual void setFIFOFilter(FLEXCAN_FLTEN a) = 0;
    virtual void setMBFilter(FLEXCAN_FLTEN a) = 0;
    virtual void setFIFOFilterRange(uint8_t f, uint32_t id1, uint32_t id2, FLEXCAN_IDE ide) = 0;
    virtual int  write(const CAN_message_t &m) = 0;
    virtual int  readFIFO(CAN_message_t &m) = 0;

    CANDevice_t device{};   // per-bus state the port needs at RX time
};

template <CAN_DEV_TABLE BUS,
          FLEXCAN_RXQUEUE_TABLE RX = RX_SIZE_256,
          FLEXCAN_TXQUEUE_TABLE TX = TX_SIZE_16>
class TeensyCAN : public TeensyCANBase {
public:
    FlexCAN_T4<BUS, RX, TX> can;

    void begin() override                        { can.begin(); }
    void setBaudRate(uint32_t baud) override     { can.setBaudRate(baud); }
    void enableFIFO() override                   { can.enableFIFO(); }
    void setFIFOFilter(FLEXCAN_FLTEN a) override { can.setFIFOFilter(a); }
    void setMBFilter(FLEXCAN_FLTEN a) override   { can.setMBFilter(a); }
    void setFIFOFilterRange(uint8_t f, uint32_t id1, uint32_t id2, FLEXCAN_IDE ide) override {
        can.setFIFOFilterRange(f, id1, id2, ide);
    }
    int write(const CAN_message_t &m) override   { return can.write(m); }
    int readFIFO(CAN_message_t &m) override      { return can.readFIFO(m); }
};