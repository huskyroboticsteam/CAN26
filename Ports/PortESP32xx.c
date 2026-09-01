/* This port is to be used with ESP32 and the ESP-IDF toolchain. It provides implementations that should
 * be compatible with any ESP32-XX SoC, but it has only been tested on the ESP32 and (TODO:) ESP32-S3.
 *
 * To use this port:
 * - This port file is automatically selected by the presence of the CONFIG_IDF_TARGET directive. The
 * ESP-IDF toolchain generates this upon running `idf.py reconfigure`).
 * - Run `idf.py menuconfig` and go to "Component config > CAN Config" to set RX/TX GPIOs and test mode.
 *
 * Important:
 * - CANHandle_t parameters are ignored. You should pass NULL to the CANHandle_t parameter of CANInit,
 * CANSend, and CANPollAndReceive.
 *     - Why? On STM32, CANHandle refers to a FDCAN_HandleTypeDef that is created by CubeMX and exists
 *       before CANInit runs. The STM32 initialization functions never rewrite the pointer's value. On
 *       ESP32, twai_new_node_onchip() *allocates* the node and returns its address via an output
 *       parameter. CANInit cannot output a new pointer value without adding extra indirection. Because
 *       ESP32 chipsets only feature one CAN controller, there is no need for a client-managed handle,
 *       so this port owns a static handle.
 */

#include "Port.h"

#if defined(CONFIG_IDF_TARGET)

#include <freertos/FreeRTOS.h>
#include <string.h>

#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#define TAG "CAN"

// ESP-IDF only maintains a FIFO of TX frame pointers. We need to store the frames in a storage pool
// until the transmission complete callback (twai_tx_cb) releases them
static twai_frame_t frame_pool[CONFIG_TX_POOL_SIZE];  // hold up to TX_POOL_LEN frames
static uint8_t data_pool[CONFIG_TX_POOL_SIZE][8];     // hold TX_POOL_LEN arrays, each holding 8 bytes
static QueueHandle_t free_q;                          // hold the available pool indexes

// Static handle that is used instead of CANHandle_t args
static twai_node_handle_t twai_handle = NULL;

// Hold CANPackets until CANPollAndReceive can return them
static QueueHandle_t rx_q;

// ESP32 SoCs only have 2 HW CAN filters; we need to save our domain bits and filter them in software
static uint8_t our_domains;

// Error callback
static uint32_t twai_err_flags;  // TODO: proper API to expose these flags
static bool twai_error_cb(twai_node_handle_t handle, const twai_error_event_data_t* edata, void* user_ctx) {
    twai_err_flags = edata->err_flags.val;
    return false;
}

// Transmission complete callback
static bool twai_tx_cb(twai_node_handle_t handle, const twai_tx_done_event_data_t* edata, void* user_ctx) {
    // the frame that was transmitted and needs to be deleted
    const twai_frame_t* tx_frame = edata->done_tx_frame;

    // pointer arithmetic to get the pool index to release
    uint8_t idx = (uint8_t)(tx_frame - frame_pool);

    BaseType_t yield_from_isr = pdFALSE;
    xQueueSendFromISR(free_q, &idx, &yield_from_isr);
    return yield_from_isr;
}

// Receive callback
static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t* edata, void* user_ctx) {
    // RX buffers
    uint8_t rx_data[8];
    twai_frame_t rx_frame = {
        .buffer = rx_data,
        .buffer_len = 8,
    };

    // read data from HW RX FIFO
    esp_err_t ret = twai_node_receive_from_isr(handle, &rx_frame);
    if (ret != ESP_OK)
        return false;

    uint8_t uuid = (rx_frame.header.id >> 3) & 0x7F;  // get 7-bit UUID
    if (uuid == 0) {
        // the UUID is 0, so hardware didn't get a UUID match; we need to check for a domain broadcast
        uint8_t frame_domains = rx_frame.header.id & 0x7;
        if ((our_domains & frame_domains) == 0)
            return false;  // none of the domains match; disregard this frame
    }

    // check DLC
    uint16_t dlc = rx_frame.header.dlc;
    if (dlc < 2 || dlc > 8)
        return false;

    // compose target device (us)
    uint16_t id = rx_frame.header.id & 0x7FF;  // get lowest 11 bits
    CANDevice_t us = {
        .peripheralDomain = id & 0x1,
        .motorDomain = (id >> 1) & 0x1,
        .powerDomain = (id >> 2) & 0x1,
        .deviceUUID = uuid,
    };

    // compose CAN packet
    CANPacket_t packet = {
        .device = us,
        .priority = id & 0x400 ? CAN_PRIORITY_LOW : CAN_PRIORITY_HIGH,
        .contentsLength = dlc - 2,
        .command = rx_data[0],
        .senderUUID = rx_data[1],
    };
    memcpy(packet.contents, rx_data + 2, dlc - 2);  // copy data bytes

    // enqueue packet to be polled later
    BaseType_t yield_from_isr = pdFALSE;
    xQueueSendFromISR(rx_q, &packet, &yield_from_isr);

    return yield_from_isr;  // signal if a higher-priority task was woken
}

uint8_t CANInit(CANHandle_t CANHandle, const CANDevice_t* device) {
    // TWAI config
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = CONFIG_TX_GPIO,
        .io_cfg.rx = CONFIG_RX_GPIO,
        .bit_timing.bitrate = 125000,  // 125 kbps bitrate
        .tx_queue_depth = CONFIG_TX_POOL_SIZE,
    };

#if CONFIG_TEST_MODE
    // if test mode is enabled, set loopback and self_test flags
    node_config.flags.enable_loopback = true;
    node_config.flags.enable_self_test = true;
    ESP_LOGW(TAG, "Test mode enabled");
#endif

    // create a new TWAI controller driver instance
    esp_err_t ret = twai_new_node_onchip(&node_config, &twai_handle);
    if (ret != ESP_OK)
        return ret;

    // register callbacks
    twai_event_callbacks_t user_cbs = {
        .on_rx_done = twai_rx_cb,
        .on_tx_done = twai_tx_cb,
        .on_error = twai_error_cb,
    };
    ret = twai_node_register_event_callbacks(twai_handle, &user_cbs, NULL);
    if (ret != ESP_OK)
        return ret;

    // initialize queues
    rx_q = xQueueCreate(CONFIG_RX_FIFO_SIZE, sizeof(CANPacket_t));
    if (!rx_q)
        return 1;
    free_q = xQueueCreate(CONFIG_TX_POOL_SIZE, sizeof(uint8_t));
    if (!free_q) {
        vQueueDelete(rx_q);
        return 1;
    }
    // fill the pool queue with available indexes (all)
    for (uint8_t i = 0; i < CONFIG_TX_POOL_SIZE; i++)
        xQueueSend(free_q, &i, 0);

    // filter 1: UUID matches our own
    uint16_t id_1 = device->deviceUUID << 3;  // filter by this device's UUID
    uint16_t mask_1 = 0x3F8;                  // mask the 7 UUID bits

    // filter 2: UUID is 0
    uint16_t id_2 = 0x000;    // filter by UUID = 0
    uint16_t mask_2 = 0x3F8;  // mask the 7 UUID bits

    // load both filters
    twai_mask_filter_config_t dual_config = twai_make_dual_filter(id_1, mask_1, id_2, mask_2, false);
    ret = twai_node_config_mask_filter(twai_handle, 0, &dual_config);
    if (ret != ESP_OK)
        return ret;

    // save our domains for software filtering in the RX callback
    our_domains = (device->powerDomain << 2) | (device->motorDomain << 1) | device->peripheralDomain;

    // start the TWAI controller
    return twai_node_enable(twai_handle);
}

uint8_t CANSend(CANHandle_t CANHandle, const CANPacket_t* packet) {
    if (packet->contentsLength > 6)
        return 1;

    // get an index from the frame/data pools to build the frame into
    uint8_t idx;
    if (xQueueReceive(free_q, &idx, 0) != pdTRUE)
        return 1;  // pool full
    twai_frame_t* frame = &frame_pool[idx];
    uint8_t* data = data_pool[idx];

    // set the data
    data[0] = packet->command;
    data[1] = packet->senderUUID;
    memcpy(data + 2, packet->contents, packet->contentsLength);

    // assemble the frame
    frame->header.dlc = packet->contentsLength + 2;
    frame->header.id = CANGetPacketHeader(packet);
    frame->buffer = data;
    frame->buffer_len = packet->contentsLength + 2;

    // queue a pointer to the TX frame; this function returns immediately
    esp_err_t ret = twai_node_transmit(twai_handle, frame, 0);
    if (ret != ESP_OK) {
        xQueueSend(free_q, &idx, 0);  // return the pool slot
        return ret;
    }

    return 0;
}

int8_t CANPollAndReceive(CANHandle_t CANHandle, CANPacket_t* packet) {
    return xQueueReceive(rx_q, packet, 0);
}

#endif  // defined(CONFIG_IDF_TARGET)
