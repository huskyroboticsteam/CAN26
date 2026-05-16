#pragme once
#include <ESP32-TWAI-CAN.hpp>

typedef struct  {
    int tx_pin;
    int rx_pin;
    CANDevice_t device;
} ESP32CANHandle_t;