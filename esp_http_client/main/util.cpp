#include <stdio.h>
#include <cstdint>
#include <utility>
#include "freertos/FreeRTOS.h"
#include "types.h"
#include "util.h"

//////////////////////////////////////////////////////////////////////
// calculate a crc16

uint16 IRAM_ATTR crc16(byte const *p, size_t len)
{
    assert(len != 0);
    uint16 crc = 0xffff;
    byte const *e = p + len;
    do {
        uint16 x = (crc >> 8) ^ *p;
        x ^= x >> 4;
        crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
    } while(++p < e);
    return crc;
}

//////////////////////////////////////////////////////////////////////
// and with copy to somewhere else

uint16 IRAM_ATTR crc16_copy(byte const *p, byte *dst, size_t len)
{
    assert(len != 0);
    uint16 crc = 0xffff;
    byte const *e = p + len;
    do {
        byte c = *p;
        uint16 x = (crc >> 8) ^ c;
        x ^= x >> 4;
        crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
        *dst++ = c;
    } while(++p < e);
    return crc;
}
