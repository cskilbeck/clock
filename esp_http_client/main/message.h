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

//////////////////////////////////////////////////////////////////////

enum hours_mode : int
{
    hours_24 = 0,
    hours_12 = 1
};

//////////////////////////////////////////////////////////////////////

#define clock_message_signature 0xDC
#define control_message_signature 0xDD

//////////////////////////////////////////////////////////////////////
// it sends this before the message

struct message_header_t
{
    byte sig;       // see #defines above
    byte length;    // sizeof derived type
    uint16 crc;     // crc of the message
} __attribute__((packed));

//////////////////////////////////////////////////////////////////////
// clock_message_t - what to display as time.

struct clock_message_t
{
    // messenger admin
    enum
    {
        signature = clock_message_signature
    };

    uint32 hours : 5;
    uint32 minutes : 6;
    uint32 seconds : 6;
    uint32 milliseconds : 10;
    uint32 pad : 5;
} __attribute__((packed));

//////////////////////////////////////////////////////////////////////
// control_message_t - any other stuff

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
} __attribute__((packed));

//////////////////////////////////////////////////////////////////////

size_t constexpr largest_message_size = sizeof(largest_type<clock_message_t, control_message_t>::type);
