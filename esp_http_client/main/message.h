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

enum event_bits : uint32
{
    send_control = 1,    // please send the control message
    control_sent = 2,    // I sent it
    send_clock = 4,      // please send the clock message
    clock_sent = 8       // I sent it
};

//////////////////////////////////////////////////////////////////////

// first 3 bytes are signature (16 bits) len (8 bits)
// then message
// then crc (16 bits)


#define clock_message_signature 0xDC
#define control_message_signature 0xDD

struct message_base_t
{
};

//////////////////////////////////////////////////////////////////////
// clock_message_t - what to display as time.
// guaranteed to arrive 1 second apart so the stm32 can use this
// to synchronize effects which happen between seconds (fading etc)

struct clock_message_t : message_base_t
{
    // messenger admin
    enum
    {
        signature = clock_message_signature,
        send_bits = send_clock,
        sent_bits = clock_sent
    };

    uint32 hours : 5;
    uint32 minutes : 6;
    uint32 seconds : 6;
    uint32 milliseconds : 10;
    uint32 pad : 5;
};

//////////////////////////////////////////////////////////////////////
// control_message_t - control display options
// Not synchronized with anything, might disrupt the clock timing
// temporarily, but not by much

struct control_message_t : message_base_t
{
    enum
    {
        signature = control_message_signature,
        send_bits = send_control,
        sent_bits = control_sent
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

size_t constexpr largest_message_size = sizeof(largest_type<clock_message_t, control_message_t>::type);
