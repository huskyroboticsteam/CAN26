#pragma once

#define CAN_UUID_BROADCAST ((CANDeviceUUID_t)0x00)
#define CAN_UUID_JETSON    ((CANDeviceUUID_t)0x01)

// BLDCs
#define CAN_UUID_BLDC_FRONT_TIRE_LEFT  ((CANDeviceUUID_t)0x30)
#define CAN_UUID_BLDC_FRONT_TIRE_RIGHT ((CANDeviceUUID_t)0x31)
#define CAN_UUID_BLDC_REAR_TIRE_LEFT   ((CANDeviceUUID_t)0x32)
#define CAN_UUID_BLDC_REAR_TIRE_RIGHT  ((CANDeviceUUID_t)0x33)

#define CAN_UUID_BLDC_BASE             ((CANDeviceUUID_t)0x34)
#define CAN_UUID_BLDC_SHOULDER         ((CANDeviceUUID_t)0x35)
#define CAN_UUID_BLDC_ELBOW            ((CANDeviceUUID_t)0x36)

#define CAN_UUID_BLDC_FOREARM          ((CANDeviceUUID_t)0x37)
#define CAN_UUID_BLDC_WRIST_LEFT       ((CANDeviceUUID_t)0x38)
#define CAN_UUID_BLDC_WRIST_RIGHT      ((CANDeviceUUID_t)0x39)

// MISC
#define CAN_UUID_TELEMETRY      ((CANDeviceUUID_t)0x50)
#define CAN_UUID_HAND           ((CANDeviceUUID_t)0x60)

#define CAN_UUID_DEBUG1      ((CANDeviceUUID_t)0x70)
#define CAN_UUID_DEBUG2      ((CANDeviceUUID_t)0x71)


#define CAN_DEVICE_JETSON ((CANDevice_t){.deviceUUID = CAN_UUID_JETSON})
