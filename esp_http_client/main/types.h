#pragma once

//////////////////////////////////////////////////////////////////////

using byte = uint8_t;

using uint64 = uint64_t;
using uint32 = uint32_t;
using uint16 = uint16_t;
using uint8 = uint8_t;

using int64 = int64_t;
using int32 = int32_t;
using int16 = int16_t;
using int8 = int8_t;

//////////////////////////////////////////////////////////////////////
// clock display options

enum colon_flash_mode
{
    none = 0,
    flash = 1,
    flash_fast = 2,
    pulse = 3
};

struct clock_options_t
{
    uint32 brightness : 6;          // 64 levels of brightness
    uint32 colon_flash_mode : 2;    // see colon_flash_mode enum
    uint32 show_seconds : 1;        // seconds on or off
    uint32 hours_12_24 : 1;         // 12/24 hour time format
    uint32 show_am_pm : 1;          // show A or P
    uint32 show_hour_ticks : 1;     // turn hour ticks on or off
    uint32 soft_seconds : 1;        // seconds fade in or just switch on
    uint32 test_display : 1;        // switch on all the leds at full brightness
};

//////////////////////////////////////////////////////////////////////
// STM32 message payload

struct message_body_t
{
    uint64 timestamp;           // # of 100uS ticks since epoch midnight 1st Jan 1970
    clock_options_t options;    // 32 option bits
    uint16 signature;           // signature must be 0xDA5C
} __attribute__((packed));

//////////////////////////////////////////////////////////////////////
// sent with crc to the STM32

struct message_t
{
    message_body_t msg;    // payload
    uint16 crc;            // 16 bit crc of previous fields
} __attribute__((packed));
