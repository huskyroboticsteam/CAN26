#include "CANPacket.h"

#include <string.h>

// There may be a cleaner way to do these tests this was the most compatible way I could devise
// Obviously on any optimization level other than O0 this gets optimized away
// Done to ensure that the pointer casts don't break things in any reasonable environment
inline static bool little_endian_bitfields() {
    union {
        int x;
        struct {
            int a : 1;
            int b : 1;
        } y;
    } tester = {1};
    return tester.y.a;
}

inline static bool little_endian() {
    union {
        int x;
        char y;
    } tester = {1};
    return tester.y;
}


#ifdef __GNUC__

#define bswap32 __builtin_bswap32
#define bswap16 __builtin_bswap16
#define clz16 __builtin_clz

#else

inline static uint32_t bswap32(uint32_t input) {
    return (input & 0xFF) << 24 | (input & 0xFF00) << 8 | (input & 0xFF0000) >> 8 | input >> 24;
}

inline static uint16_t bswap16(uint16_t input) {
    return (input & 0xFF) << 8 | input >> 8;
}

inline static uint8_t clz16(uint16_t input) {
    uint8_t count = 0;
    for (int i = 0; i < 16; ++i, input <<= 1) {
        count += (input & 0x8000) != 0;
    }
    return count;
}

#endif

uint16_t CANGetPacketHeader(CANPacket_t *packet) {
    uint16_t device;
    if (sizeof(CANDevice_t) == sizeof(uint16_t) && little_endian_bitfields()) {
        memcpy(&device, &packet->device, sizeof(uint16_t));
        device &= 0x3FF;
    } else {
        // backup code for big endian processors
        device = (packet->device.deviceUUID << 3) +
                 (packet->device.peripheralDomain << 2) +
                 (packet->device.powerDomain << 1) +
                 (packet->device.motorDomain);
    }
    return (~packet->priority << 10) + device;
}

uint8_t CANGetDlc(CANPacket_t *packet) {
    return packet->contentsLength + 2;
}

uint8_t *CANGetData(CANPacket_t *packet) {
    return (uint8_t *)&packet->command;
}

uint32_t CANLoadUInt32(uint8_t *ptr) {
    uint32_t result;
    memcpy(&result, ptr, sizeof(uint32_t));
    if (!little_endian()) {
        result = bswap32(result);
    }
    return result;
}

int32_t CANLoadInt32(uint8_t *ptr) {
    return (int32_t)CANLoadUInt32(ptr);
}

uint32_t CANLoadUInt24(uint8_t *ptr) {
    return ptr[2] << 16 | ptr[1] << 8 | ptr[0];
}

int32_t CANLoadInt24(uint8_t *ptr) {
    uint32_t zeroExtended = CANLoadUInt24(ptr);
    uint32_t negative = -(zeroExtended & 0x800);
    uint32_t upperBits = 0xF000 & negative;
    return (int32_t)(zeroExtended | upperBits);
}

uint16_t CANLoadUInt16(uint8_t *ptr) {
    uint16_t result;
    memcpy(&result, ptr, sizeof(uint16_t));
    if (!little_endian()) {
        result = bswap16(result);
    }
    return result;
}

int16_t CANLoadInt16(uint8_t *ptr) {
    return (int16_t)CANLoadUInt16(ptr);
}

// Assumes sizeof(float) == sizeof(uint32_t)
float CANLoadFloat32(uint8_t *ptr) {
    uint32_t intVal = CANLoadUInt32(ptr);
    float floatVal;
    memcpy(&floatVal, &intVal, sizeof(float));
    return floatVal;
}

float CANLoadBFloat24(uint8_t *ptr) {
    uint32_t intVal = CANLoadUInt24(ptr) << 8;
    float floatVal;
    memcpy(&floatVal, &intVal, sizeof(float));
    return floatVal;
}

float CANLoadBFloat16(uint8_t *ptr) {
    uint32_t intVal = (uint32_t)CANLoadUInt16(ptr) << 16;
    float floatVal;
    memcpy(&floatVal, &intVal, sizeof(float));
    return floatVal;
}

float CANLoadFloat16(uint8_t *ptr) {
    uint16_t intVal = CANLoadUInt16(ptr);
    int16_t exp = ((intVal & 0x7C00) >> 10) - 15;
    uint8_t sign = intVal >= 0x8000;
    uint16_t mantissa = intVal & 0x3FF;

    if (exp == 0x10) {
        exp = 128;
    } else if (exp == -15) {
        if (mantissa == 0) {
            exp = -127;
        } else {
            uint8_t lz = clz16(mantissa) - 21;
            exp -= lz - 1;
            mantissa <<= lz;
            mantissa &= 0x3FF;
        }
    }

    uint32_t newIntVal = sign << 31 | (exp + 127) << 23 | mantissa << 13;
    float result;
    memcpy(&result, &newIntVal, sizeof(float));
    return result;
}

float CANLoadUNorm16(uint8_t *ptr) {
    uint16_t intVal = CANLoadUInt16(ptr);
    return intVal / 65535.0f;
}

float CANLoadUNorm8(uint8_t *ptr) {
    return *ptr / 255.0f;
}

void CANStoreUInt32(uint8_t *ptr, uint32_t value) {
    if (!little_endian()) {
        value = bswap32(value);
    }
    memcpy(ptr, &value, sizeof(uint32_t));
}

void CANStoreInt32(uint8_t *ptr, int32_t value) {
    CANStoreUInt32(ptr, (uint32_t)value);
}

void CANStoreUInt24(uint8_t *ptr, int32_t value) {
    if (!little_endian()) {
        value = bswap32(value);
    }
    memcpy(ptr, (char *)&value + 1, 3);
}

void CANStoreInt24(uint8_t *ptr, int32_t value) {
    CANStoreUInt24(ptr, (uint32_t)value);
}

void CANStoreUInt16(uint8_t *ptr, uint16_t value) {
    if (!little_endian()) {
        value = bswap16(value);
    }
    memcpy(ptr, &value, sizeof(uint16_t));
}

void CANStoreInt16(uint8_t *ptr, int16_t value) {
    CANStoreUInt16(ptr, (uint16_t)value);
}

// Assumes sizeof(float) == sizeof(uint32_t)
void CANStoreFloat32(uint8_t *ptr, float value) {
    uint32_t intVal;
    memcpy(&intVal, &value, sizeof(float));
    CANStoreUInt32(ptr, intVal);
}

void CANStoreBFloat24(uint8_t *ptr, float value) {
    uint32_t intVal;
    memcpy(&intVal, &value, sizeof(float));
    uint32_t rounded = (intVal >> 8) + ((intVal >> 7) & 1);
    CANStoreUInt24(ptr, rounded);
}

void CANStoreBFloat16(uint8_t *ptr, float value) {
    uint32_t intVal;
    memcpy(&intVal, &value, sizeof(float));
    uint16_t rounded = (uint16_t)((intVal >> 16) + ((intVal >> 15) & 1));
    CANStoreUInt16(ptr, rounded);
}

void CANStoreFloat16(uint8_t *ptr, float value) {
    uint32_t intVal;
    memcpy(&intVal, &value, sizeof(float));
    uint8_t sign = intVal >> 31;
    int16_t exp = ((intVal & 0x7F800000) >> 23) - 127;
    uint32_t mantissa = intVal & 0x7FFFFF;

    uint8_t newExp;
    uint16_t newMantissa;
    if (exp >= 0x10) {
        if (value == value) {
            newExp = 0x1F;
            newMantissa = 0;
        } else {
            sign = 0;
            newExp = 0x1F;
            newMantissa = 0x3FF;
        }
    } else if (exp < -25) { // Clamped to 0
        newExp = 0;
        newMantissa = 0;
    } else if (exp < -14) {
        mantissa |= 1 << 23;
        // round to nearest, ties away from 0
        newMantissa = (mantissa >> (13 + -14 - exp)) + ((mantissa >> (12 + -14 - exp)) & 1);
        newExp = 0;
    } else {
        newExp = exp + 15;
        // round to nearest, ties away from 0
        newMantissa = (mantissa >> 13) + ((mantissa >> 12) & 1);

    }
    uint16_t resultValue = sign << 15 | newExp << 10 | newMantissa;
    CANStoreUInt16(ptr, resultValue);
}

void CANStoreUNorm24(uint8_t *ptr, float value) {
    if (value > 1.0) {
        value = 1.0;
    } else if (value < 0.0) {
        value = 0.0;
    }
    value *= (float)(((uint32_t)1 << 24) - 1);
    uint32_t intVal = (uint32_t)(value + 0.5f);
    if (intVal > 0xFFFFFF) intVal = 0xFFFFFF;
    CANStoreUInt24(ptr, intVal);
}

void CANStoreUNorm16(uint8_t *ptr, float value) {
    if (value > 1.0) {
        value = 1.0;
    } else if (value < 0.0) {
        value = 0.0;
    }
    value *= 65535.f;
    // round to nearest, ties away from 0
    uint32_t intVal = (uint32_t)(value + 0.5f);
    if (intVal > 0xFFFF) intVal = 0xFFFF;
    CANStoreUInt16(ptr, (uint16_t)intVal);
}

void CANStoreUNorm8(uint8_t *ptr, float value) {
    if (value > 1.0) {
        value = 1.0;
    } else if (value < 0.0) {
        value = 0.0;
    }
    value *= 255.f;
    // round to nearest, ties away from 0
    uint16_t intVal = (uint16_t)(value + 0.5f);
    if (intVal > 0xFF) intVal = 0xFF;
    *ptr = (uint8_t)intVal;
}
