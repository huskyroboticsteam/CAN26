#pragma once

#define CAN_UUID_BROADCAST ((CANDeviceUUID_t)0x00)
#define CAN_UUID_JETSON    ((CANDeviceUUID_t)0x01)

#define CAN_DEVICE_JETSON ((CANDevice_t){.deviceUUID = CAN_UUID_JETSON})
