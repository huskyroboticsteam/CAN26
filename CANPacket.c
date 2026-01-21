#include "CANPacket.h"
#include "CANHelpers.h"

#include <string.h>

/**
 * Serializes the CANPacket destination device and priority
 * Result is to be used in the 11 bit portion of the protocol
 *
 * The packet pointer is assumed to be a valid packet
 */
uint16_t CANGetPacketHeader(CANPacket_t *packet) {
    uint16_t device;
    if (sizeof(CANDevice_t) == sizeof(uint16_t) && little_endian_bitfields()) {
        // This branch of the if is taken in most applicable environments
        memcpy(&device, &packet->device, sizeof(uint16_t));
        
        // Padding contents are implementation defined, this line ensures they are excluded
        device &= 0x3FF;
    } else {
        // backup code for big endian processors or if the bit field is not condensed
        device = (packet->device.deviceUUID << 3) +
                 (packet->device.peripheralDomain << 2) +
                 (packet->device.powerDomain << 1) +
                 (packet->device.motorDomain);
    }
    // Note that priority is inverted from the actual value
    return (!packet->priority << 10) + device;
}

/**
 * Returns the data length code that should be used for the can packet
 * Note that the contents length of the packet is 2 less than the actual data length code
 */
uint8_t CANGetDlc(CANPacket_t *packet) {
    return packet->contentsLength + 2;
}

/**
 * Returns a pointer to the start of the (up to) 8 byte data used in the can packet
 */
uint8_t *CANGetData(CANPacket_t *packet) {
    return (uint8_t *)&packet->command;
}

/**
 * Returns the 32 bit unsigned value stored at the given memory location
 * Enforces LE and the location is allowed to be misaligned
 */
uint32_t CANLoadUInt32(uint8_t *ptr) {
    uint32_t result;
    memcpy(&result, ptr, sizeof(uint32_t));
    if (!little_endian()) {
        result = bswap32(result);
    }
    return result;
}

/**
 * Returns the 32 bit signed value stored in the given memory location
 * Enforces LE and the location is allowed to be misaligned
 */
int32_t CANLoadInt32(uint8_t *ptr) {
    return (int32_t)CANLoadUInt32(ptr);
}

/**
 * Returns the 24 bit unsigned value stored in the given memory location
 * Enforces LE and the location is allowed to be misaligned
 */
uint32_t CANLoadUInt24(uint8_t *ptr) {
    return ptr[2] << 16 | ptr[1] << 8 | ptr[0];
}

/**
 * Returns the 24 bit unsigned value stored in the given memory location
 * Enforces LE and the location is allowed to be misaligned
 * The value is sign extended to 32 bits
 */
int32_t CANLoadInt24(uint8_t *ptr) {
    uint32_t zeroExtended = CANLoadUInt24(ptr);
    uint32_t negative = -(zeroExtended & 0x800);
    // Sign extended upper bits of the result
    uint32_t upperBits = 0xF000 & negative;
    return (int32_t)(zeroExtended | upperBits);
}

/**
 * Returns the 16 bit unsigned value stored in the given memory location
 * Enforces LE and the location is allowed to be misaligned
 */
uint16_t CANLoadUInt16(uint8_t *ptr) {
    uint16_t result;
    memcpy(&result, ptr, sizeof(uint16_t));
    if (!little_endian()) {
        result = bswap16(result);
    }
    return result;
}

/**
 * Returns the 16 bit signed value stored in the given memory location
 * Enforces LE and the location is allowed to be misaligned
 */
int16_t CANLoadInt16(uint8_t *ptr) {
    return (int16_t)CANLoadUInt16(ptr);
}

/**
 * Returns the 32 bit float value stored in the given memory location
 * Enforces LE and the location is allowed to be misaligned
 * Assumes floats are 4 bytes in size
 * Assumes all parties use IEEE 754 single precision floats
 * Assumes floats use the same endianness as ints
 */
float CANLoadFloat32(uint8_t *ptr) {
    uint32_t intVal = CANLoadUInt32(ptr);
    float floatVal;
    memcpy(&floatVal, &intVal, sizeof(float));
    return floatVal;
}

/**
 * Returns the 24 bit float value stored in the given memory location
 * The 24 bit floats are truncated 32 bit floats
 * Enforces LE and the location is allowed to be misaligned
 * Assumes floats are 4 bytes in size
 * Assumes all parties use IEEE 754 single precision floats
 * Assumes floats use the same endianness as ints
 */
float CANLoadBFloat24(uint8_t *ptr) {
    uint32_t intVal = CANLoadUInt24(ptr) << 8;
    float floatVal;
    memcpy(&floatVal, &intVal, sizeof(float));
    return floatVal;
}

/**
 * Returns the 16 bit brain float value stored in the given memory location
 * 16 bit brain floats are truncated 32 bit floats
 * Enforces LE and the location is allowed to be misaligned
 * Assumes floats are 4 bytes in size
 * Assumes all parties use IEEE 754 single precision floats
 * Assumes floats use the same endianness as ints
 */
float CANLoadBFloat16(uint8_t *ptr) {
    uint32_t intVal = (uint32_t)CANLoadUInt16(ptr) << 16;
    float floatVal;
    memcpy(&floatVal, &intVal, sizeof(float));
    return floatVal;
}

/**
 * Returns the 16 bit float value stored in the given memory location
 * 16 bit floats are IEEE 754 half precision floats
 * Correctly handles subnormals
 * Enforces LE and the location is allowed to be misaligned
 * Assumes floats are 4 bytes in size
 * Assumes all parties use IEEE 754 single precision floats
 * Assumes floats use the same endianness as ints
 */
float CANLoadFloat16(uint8_t *ptr) {
    uint16_t intVal = CANLoadUInt16(ptr);
    int16_t exp = ((intVal & 0x7C00) >> 10) - 15;
    uint8_t sign = intVal >= 0x8000;
    uint16_t mantissa = intVal & 0x3FF;

    if (exp == 0x10) {
        // handle inf/nan
        exp = 128;
    } else if (exp == -15) {
        // handle subnormals
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

/**
 * Returns the 24 bit unsigned normalized value stored at the given memory location
 * 24 bit UNorm values map the range 0x000000-0xFFFFFF to the range 0.0-1.0
 * Enforces LE and the location is allowed to be misaligned
 */
float CANLoadUNorm24(uint8_t *ptr) {
    uint32_t intVal = CANLoadUInt24(ptr);
    return intVal / 16777215.0f;
}

/**
 * Returns the 16 bit unsigned normalized value stored in the given memory location
 * 16 bit UNorm values map the range 0-65535 to the range 0.0-1.0
 * Enforces LE and the location is allowed to be misaligned
 */
float CANLoadUNorm16(uint8_t *ptr) {
    uint16_t intVal = CANLoadUInt16(ptr);
    return intVal / 65535.0f;
}

/**
 * Returns the 8 bit unsigned normalized value stored in the given memory location
 * 8 bit UNorm values map the range 0-255 to the range 0.0-1.0
 */
float CANLoadUNorm8(uint8_t *ptr) {
    return *ptr / 255.0f;
}

/**
 * Stores a 32 bit unsigned integer into the given memory location
 * Enforces LE and the location is allowed to be misaligned
 */
void CANStoreUInt32(uint8_t *ptr, uint32_t value) {
    if (!little_endian()) {
        value = bswap32(value);
    }
    memcpy(ptr, &value, sizeof(uint32_t));
}

/**
 * Stores a 32 bit signed integer into the given memory location
 * Enforces LE and the location is allowed to be misaligned
 */
void CANStoreInt32(uint8_t *ptr, int32_t value) {
    CANStoreUInt32(ptr, (uint32_t)value);
}

/**
 * Stores a 24 bit unsigned integer into the given memory location
 * Enforces LE and the location is allowed to be misaligned
 * Ignores the upper 8 bits of the 32 bit value
 * Assumes sizeof(int32_t) == 4
 */
void CANStoreUInt24(uint8_t *ptr, int32_t value) {
    if (!little_endian()) {
        value = bswap32(value);
    }
    memcpy(ptr, (char *)&value + 1, 3);
}

/**
 * Stores a 24 bit signed integer into the given memory location
 * Enforces LE and the location is allowed to be misaligned
 * Ignores the upper 8 bits of the 32 bit value
 * Assumes sizeof(int32_t) == 4
 */
void CANStoreInt24(uint8_t *ptr, int32_t value) {
    CANStoreUInt24(ptr, (uint32_t)value);
}

/**
 * Stores a 16 bit unsigned integer into the given memory location
 * Enforces LE and the location is allowed to be misaligned
 */
void CANStoreUInt16(uint8_t *ptr, uint16_t value) {
    if (!little_endian()) {
        value = bswap16(value);
    }
    memcpy(ptr, &value, sizeof(uint16_t));
}

/**
 * Stores a 16 bit signed integer into the given memory location
 * Enforces LE and the location is allowed to be misaligned
 */
void CANStoreInt16(uint8_t *ptr, int16_t value) {
    CANStoreUInt16(ptr, (uint16_t)value);
}

/**
 * Stores a 32 bit float into the given memory location
 * Enforces LE and the location is allowed to be misaligned
 * Assumes sizeof(float) == sizeof(uint32_t)
 * Assumes all parties use IEEE 754 single precision floats
 * Assumes the float endianness is the same as int endianness
 */
void CANStoreFloat32(uint8_t *ptr, float value) {
    uint32_t intVal;
    memcpy(&intVal, &value, sizeof(float));
    CANStoreUInt32(ptr, intVal);
}

/**
 * Stores a 24 bit float into the given memory location
 * The 24 bit float is a truncated IEEE 754 single precision float
 * Non standard float format
 * Uses the round to nearest (ties away from 0) rounding mode
 * Clamps out of range values
 * Enforces LE and the location is allowed to be misaligned
 * Assumes sizeof(float) == sizeof(uint32_t)
 * Assumes all parties use IEEE 754 single precision floats
 * Assumes the float endianness is the same as int endianness
 */
void CANStoreBFloat24(uint8_t *ptr, float value) {
    uint32_t intVal;
    memcpy(&intVal, &value, sizeof(float));
    uint32_t rounded = (intVal >> 8) + ((intVal >> 7) & 1);
    CANStoreUInt24(ptr, rounded);
}

/**
 * Stores a 16 bit brain float into the given memory location
 * Brain floats are truncated IEEE 754 single precision floats
 * Uses the round to nearest (ties away from 0) rounding mode
 * Clamps out of range values
 * Enforces LE and the location is allowed to be misaligned
 * Assumes sizeof(float) == sizeof(uint32_t)
 * Assumes all parties use IEEE 754 single precision floats
 * Assumes the float endianness is the same as int endianness
 */
void CANStoreBFloat16(uint8_t *ptr, float value) {
    uint32_t intVal;
    memcpy(&intVal, &value, sizeof(float));
    uint16_t rounded = (uint16_t)((intVal >> 16) + ((intVal >> 15) & 1));
    CANStoreUInt16(ptr, rounded);
}

/**
 * Stores a 16 bit float into the given memory location
 * 16 bit floats are IEEE 754 half precision floats
 * Uses the round to nearest (ties away from 0) rounding mode
 * Enforces LE and the location is allowed to be misaligned
 * Assumes sizeof(float) == sizeof(uint32_t)
 * Assumes all parties use IEEE 754 single precision floats
 * Assumes the float endianness is the same as int endianness
 */
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
            // Clamp to infinity
            newExp = 0x1F;
            newMantissa = 0;
        } else {
            // NaN
            sign = 0;
            newExp = 0x1F;
            newMantissa = 0x3FF;
        }
    } else if (exp < -25) { // Clamped to 0
        newExp = 0;
        newMantissa = 0;
    } else if (exp < -14) {
        // subnormal in half precision
        mantissa |= 1 << 23;
        // round to nearest, ties away from 0
        newMantissa = (mantissa >> (13 + -14 - exp)) + ((mantissa >> (12 + -14 - exp)) & 1);
        newExp = 0;
    } else {
        // normal number
        newExp = exp + 15;
        // round to nearest, ties away from 0
        newMantissa = (mantissa >> 13) + ((mantissa >> 12) & 1);

    }
    uint16_t resultValue = sign << 15 | newExp << 10 | newMantissa;
    CANStoreUInt16(ptr, resultValue);
}

/**
 * Stores a 24 bit unsigned normalized value into the given memory location
 * 24 bit UNorms map the range 0x000000-0xFFFFFF into the range 0.0-1.0
 * Rounds to nearest representable value, with ties away from 0
 * Clamps out of range values
 * Enforces LE and the location is allowed to be misaligned
 */
void CANStoreUNorm24(uint8_t *ptr, float value) {
    if (value > 1.0) {
        value = 1.0;
    } else if (value < 0.0) {
        value = 0.0;
    }
    value *= (float)(((uint32_t)1 << 24) - 1);
    // 0.5f is for rounding
    uint32_t intVal = (uint32_t)(value + 0.5f);
    if (intVal > 0xFFFFFF) intVal = 0xFFFFFF;
    CANStoreUInt24(ptr, intVal);
}

/**
 * Stores a 16 bit unsigned normalized value into the given memory location
 * 16 bit UNorms map the range 0-65535 into the range 0.0-1.0
 * Rounds to nearest representable value, with ties away from 0
 * Clamps out of range values
 * Enforces LE and the location is allowed to be misaligned
 */
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

/**
 * Stores an 8 bit unsigned normalized value into the given memory location
 * 8 bit UNorms map the range 0-255 into the range 0.0-1.0
 * Rounds to nearest representable value, with ties away from 0
 * Clamps out of range values
 */
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
