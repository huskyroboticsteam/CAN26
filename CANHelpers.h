#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * This header consists of various helper macros and functions
 * These are for use in other parts of the library
 * There should probably be no reason to include this file directly
 *
 * Most of this is for handling the potential for obscure and unorthodox architectures
 * Realisticly it does not matter unless we switch to big endian, but some of it is necessary for c++ reasons
 */

/*
 * These macros are to find the way to define a small enum in the environment
 * Small enums save a net of 5 bytes per packet
 * These macros may not be necessary if we restrict our target more
 */
#if __STDC_VERSION__ >= 202311L || __cplusplus >= 201103L
    #define SMALL_ENUM typedef enum : uint8_t
#elif defined(__GNUC__)
    #define SMALL_ENUM typedef enum __attribute__((__packed__))
#else
    // Small enums don't exist in this case
    #define SMALL_ENUM typedef enum
#endif

// There may be a cleaner way to do these tests this was the most compatible way I could devise
// Obviously on any optimization level other than O0 this gets optimized away
// Done to ensure that the memcpys don't break things in any reasonable environment

/**
 * Indicates if bit fields are ordered from lsb to msb (generally true on LE architectures)
 */
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

/**
 * Indicates whether the platform is LE
 * Non LE architectures are assumed to be BE
 */
inline static bool little_endian() {
    union {
        int x;
        char y;
    } tester = {1};
    return tester.y;
}


#ifdef __GNUC__

/**
 * Inverses the order of the bytes contained in a 32 bit number
 */
#define bswap32 __builtin_bswap32

/**
 * Swaps the order of the bytes contained in a 16 bit number
 */
#define bswap16 __builtin_bswap16

/**
 * Counts the number of leading zeros in a 16 bit number
 */
#define clz16 __builtin_clz

#else

// Backup versions for compilers like msvc
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

