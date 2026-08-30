idf_component_register(SRCS "CANPacket.c" "./Ports/PortESP32xx.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_twai)