#pragma once

//////////////////////////////////////////////////////////////////////
// clock display options

enum colon_flash_mode : int
{
    off = 0,
    solid = 1,
    flash = 2,
    pulse = 3
};

enum hours_mode : int
{
    hours_24 = 0,
    hours_12 = 1
};

//////////////////////////////////////////////////////////////////////

// first 3 bytes are signature (16 bits) len (8 bits)
// then message
// then crc (16 bits)


#define clock_message_signature 0xDC
#define control_message_signature 0xDD

//////////////////////////////////////////////////////////////////////
// clock_message_t - what to display as time.
// guaranteed to arrive 1 second apart so the stm32 can use this
// to synchronize effects which happen between seconds (fading etc)

struct clock_message_t
{
    enum
    {
        signature = clock_message_signature
    };

    uint8 digit[7];    // ascii
    uint8 seconds;     // seconds tickmark count
};

//////////////////////////////////////////////////////////////////////
// control_message_t - control display options
// Not synchronized with anything, might disrupt the clock timing
// temporarily, but not by much

struct control_message_t
{
    enum
    {
        signature = control_message_signature
    };

    uint16 brightness : 6;          // 64 levels of brightness
    uint16 digits_enabled : 7;      // which digits enabled
    uint16 colon_flash_mode : 2;    // see colon_flash_mode enum
    uint16 hours_12_24 : 1;         // 12/24 hour time format

    uint16 show_seconds : 1;       // seconds digits on or off
    uint16 soft_seconds : 1;       // fade in the second tickmarks
    uint16 show_hour_ticks : 1;    // turn hour ticks on or off
    uint16 test_display : 1;       // switch on all the leds at full brightness
    uint16 pad : 12;
};
