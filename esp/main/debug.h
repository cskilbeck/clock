#pragma once

#include "driver/gpio.h"
#include "esp8266/gpio_struct.h"

void debug_init();

//////////////////////////////////////////////////////////////////////

inline void debug_set(gpio_num_t n)
{
    gpio_set_level(n, 1);
}

//////////////////////////////////////////////////////////////////////

inline void debug_clear(gpio_num_t n)
{
    gpio_set_level(n, 0);
}

//////////////////////////////////////////////////////////////////////

inline void debug_toggle(gpio_num_t n)
{
    gpio_set_level(n, !gpio_get_level(n));
}

//////////////////////////////////////////////////////////////////////

enum class debug_led_state
{
    off = 0,
    on = 1
};

//////////////////////////////////////////////////////////////////////

enum class debug_color
{
    off = ~0,
    red = ~1,
    green = ~2,
    yellow = ~3,
    blue = ~4,
    magenta = ~5,
    cyan = ~6,
    white = ~7
};

//////////////////////////////////////////////////////////////////////
// set debug led color permanently

void debug_set_color(debug_color c);

//////////////////////////////////////////////////////////////////////
// switch debug led on or off

void debug_switch(debug_led_state new_state);

//////////////////////////////////////////////////////////////////////
// flash debug led between two colors at some speed some # of times (0 = forever)

void debug_flash(debug_color a, debug_color b, int period_frames, int num_flashes = 0);

//////////////////////////////////////////////////////////////////////
// call this from the main loop

void debug_update();
