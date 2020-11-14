#pragma once

//////////////////////////////////////////////////////////////////////
// clock display options

enum colon_flash_mode
{
    none = 0,
    flash = 1,
    flash_fast = 2,
    pulse = 3
};

//////////////////////////////////////////////////////////////////////
// STM32 message payload, 128 bits

struct message_t
{
    // 0..7
    uint8 digit[7];    // ascii
    uint8 seconds;     // seconds tickmark count

    // 8..9
    uint16 brightness : 6;          // 64 levels of brightness
    uint16 colon_flash_mode : 2;    // see colon_flash_mode enum
    uint16 digits_enabled : 7;      // which digits enabled
    uint16 hours_12_24 : 1;         // 12/24 hour time format

    // 10..11
    uint16 show_seconds : 1;       // seconds digits on or off
    uint16 soft_seconds : 1;       // fade in the second tickmarks
    uint16 show_hour_ticks : 1;    // turn hour ticks on or off
    uint16 test_display : 1;       // switch on all the leds at full brightness

    // 12..13
    uint16 signature;    // must be 0xDA5C

    // 14..15
    uint16 crc;    // must be last - see crc16()
};

static_assert(sizeof(message_t) == 16, "Message must be 128 bits");
