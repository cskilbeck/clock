//////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>
#include <new>
#include <type_traits>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "util.h"
#include "random.h"
#include "color.h"
#include "settings.h"
#include "effect.h"
#include "debug.h"
#include "power.h"

//////////////////////////////////////////////////////////////////////

static char const *TAG = "effect";

static int global_speed = 128;
static int frames = 0;

color *effect::rgb_buffer = null;
effect *effect::global_effect = null;

// color global_dimmer(255, 220, 235);
color global_dimmer(255, 255, 255);

//////////////////////////////////////////////////////////////////////

extern byte const gammaTable[256] = { 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                                      0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3,
                                      3,   3,   3,   3,   4,   4,   4,   4,   4,   5,   5,   5,   5,   6,   6,   6,   6,   7,   7,   7,   7,   8,   8,   8,   9,   9,
                                      9,   10,  10,  10,  11,  11,  11,  12,  12,  13,  13,  13,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,  19,  20,  20,
                                      21,  21,  22,  22,  23,  24,  24,  25,  25,  26,  27,  27,  28,  29,  29,  30,  31,  32,  32,  33,  34,  35,  35,  36,  37,  38,
                                      39,  39,  40,  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  50,  51,  52,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,
                                      64,  66,  67,  68,  69,  70,  72,  73,  74,  75,  77,  78,  79,  81,  82,  83,  85,  86,  87,  89,  90,  92,  93,  95,  96,  98,
                                      99,  101, 102, 104, 105, 107, 109, 110, 112, 114, 115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
                                      144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175, 177, 180, 182, 184, 186, 189, 191, 193, 196, 198,
                                      200, 203, 205, 208, 210, 213, 215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255 };

//////////////////////////////////////////////////////////////////////

struct value
{
    byte val;
    int8 vel;
    color c[2];

    enum
    {
        in_progress = 0,
        hit_top = 1,
        hit_bottom = 2
    };

    void init(color color1, color color2)
    {
        val = 0;
        vel = 16;
        c[0] = color1;
        c[1] = color2;
    }

    void randomize()
    {
        val = 0;
        vel = 0;
        c[0].randomize();
        c[1].randomize();
    }

    static int adjust_speed(int s)
    {
        return ((s * global_speed) >> 16) + 2;
    }

    static int speed(speed_range const &range, int turbo = 32)
    {
        return adjust_speed(range.get() * turbo);
    }

    static int fixed_speed(speed_range const &range, int turbo = 32)
    {
        return adjust_speed(((range.low + range.high) / 2) * turbo);
    }

    int update(speed_range const &range, int turbo = 32)
    {
        int v = val;
        int r = in_progress;
        v += vel;
        if(v <= 0) {
            v = 0;
            vel = speed(range, turbo);
            r = hit_bottom;
        } else if(v >= 255) {
            v = 255;
            vel = -speed(range, turbo);
            r = hit_top;
        }
        val = v;
        return r;
    }

    int update_fixed(speed_range const &range)
    {
        int v = val;
        int r = in_progress;
        v += vel;
        if(v <= 0) {
            v = 0;
            vel = fixed_speed(range);
            r = hit_bottom;
        } else if(v >= 255) {
            v = 255;
            vel = -fixed_speed(range);
            r = hit_top;
        }
        val = v;
        return r;
    }

    int update_rand(color_range const &color_range, speed_range const &speed_range, int turbo = 32)
    {
        int r = update(speed_range, turbo);
        if(r == hit_top) {
            c[0] = color_range.get();
        } else if(r == hit_bottom) {
            c[1] = color_range.get();
        }
        return r;
    }

    int update_random(speed_range const &range)
    {
        int r = update(range);
        if(r == hit_top) {
            c[0].randomize();
        } else if(r == hit_bottom) {
            c[1].randomize();
        }
        return r;
    }

    color get_color()
    {
        return lerp(c[0], c[1], val);
    }
};

//////////////////////////////////////////////////////////////////////

color gamma_correct(color c)
{
    return color(gammaTable[(int(c.r) * global_dimmer.r) >> 8], gammaTable[(int(c.g) * global_dimmer.g) >> 8], gammaTable[(int(c.b) * global_dimmer.b) >> 8]);
}

//////////////////////////////////////////////////////////////////////

color effect::get_color_parameter(int index)
{
    return color_ranges[index].get();
}

//////////////////////////////////////////////////////////////////////

byte effect::get_speed_parameter(int index)
{
    return speed_ranges[index].get();
}

//////////////////////////////////////////////////////////////////////

void effect::set_pixel(color c, int led)
{
    rgb_buffer[led] = gamma_correct(c);
}

//////////////////////////////////////////////////////////////////////

void effect::set_pixels(color c, int led, int n, int stride)
{
    if(n == 0) {
        return;
    }
    color t = gamma_correct(c);
    int r = t.r;
    int g = t.g;
    int b = t.b;
    byte *q = (byte *)(rgb_buffer + led);
    stride *= 3;
    do {
        q[0] = r;
        q[1] = g;
        q[2] = b;
        q += stride;
    } while(--n != 0);
}

//////////////////////////////////////////////////////////////////////

void effect::init()
{
}

//////////////////////////////////////////////////////////////////////

void effect::update()
{
}

//////////////////////////////////////////////////////////////////////

void effect::cls(color c)
{
    set_pixels(c, 0, num_leds);
}

//////////////////////////////////////////////////////////////////////

struct red_effect : effect
{
    void init()
    {
        cls(0xff0000);
    }
};

//////////////////////////////////////////////////////////////////////

struct green_effect : effect
{
    void init()
    {
        cls(0x00ff00);
    }
};

//////////////////////////////////////////////////////////////////////

struct blue_effect : effect
{
    void init()
    {
        cls(0x0000ff);
    }
};

//////////////////////////////////////////////////////////////////////

struct yellow_effect : effect
{
    void init()
    {
        cls(0xfff800);
    }
};

//////////////////////////////////////////////////////////////////////

struct cyan_effect : effect
{
    void init()
    {
        cls(0x00ffff);
    }
};

//////////////////////////////////////////////////////////////////////

struct magenta_effect : effect
{
    void init()
    {
        cls(0xff00ff);
    }
};

//////////////////////////////////////////////////////////////////////

struct orange_effect : effect
{
    void init()
    {
        cls(0xff6800);
    }
};

//////////////////////////////////////////////////////////////////////

struct dim_effect : effect
{
    void init()
    {
        val = 0;
    }

    void update()
    {
        val += 68;
        uint32 c = (val / 16) & 0xff;
        c = c | (c << 8) | (c << 16);
        cls(c);
    }
    int val;
};

//////////////////////////////////////////////////////////////////////

struct white_effect : effect
{
    void init()
    {
        cls(0xffffff);
    }
};

//////////////////////////////////////////////////////////////////////

struct candy_effect : effect
{
    void init()
    {
        for(int i = 0; i < num_leds; ++i) {
            value &v = w[i];
            v.randomize();
        }
    }

    void update()
    {
        int led = 0;
        color_range r;
        r.low = color(0, 0, 0);
        r.high = color(255, 255, 255);
        for(int i = 0; i < num_leds; ++i) {
            value &v = w[i];
            v.update_rand(r, speed_ranges[0]);
            set_pixel(v.get_color(), led++);
        }
    }

    value w[num_leds];
};

//////////////////////////////////////////////////////////////////////

struct sine_effect : effect
{
    void init()
    {
        for(int i = 0; i < 4; ++i) {
            c[i].init(color::random(), color::random());
            c[i].vel = 20;
        }
        v[0].init(0xff0000, 0x00ff00);
        v[1].init(0x0000ff, 0xffff00);
        v[0].vel = 4;
        v[1].vel = 4;
    }

    void update()
    {
        for(int i = 0; i < 4; ++i) {
            c[i].update_random(speed_ranges[0]);
        }
        v[0].c[0] = c[0].get_color();
        v[0].c[1] = c[1].get_color();
        v[1].c[0] = c[2].get_color();
        v[1].c[1] = c[3].get_color();

        v[0].update(speed_ranges[1]);
        v[1].update(speed_ranges[2]);

        int base = frames * 16;    // later, sin(frame)
        int t = 32;
        for(int i = 0; i < num_leds; ++i) {
            set_pixel(lerp(v[0].get_color(), v[1].get_color(), (sin(base) >> 1) + 128), i);
            base += t;
        }
    }

    value c[4];
    value v[2];
};

//////////////////////////////////////////////////////////////////////

struct spark_effect : effect
{
    void init()
    {
        n = 0;
        cls(color_ranges[0].low);
    }

    void update()
    {
        set_pixel(color_ranges[0].low, n);
        n = rand() % num_leds;
        set_pixel(color_ranges[1].get(), n);
    }

    int n;
};

//////////////////////////////////////////////////////////////////////
// pulse between two colours

struct pulse_effect : effect
{
    void init()
    {
        for(int i = 0; i < 6; ++i) {
            c[i] = get_color_parameter(i);
        }
    }

    void update()
    {
        // clipped triangle wave
        int speed = speed_ranges[0].mid();
        int y = abs(((frames * speed >> 5) & 1023) - 512) - 128;
        if(y < 0) {
            y = 0;
        } else if(y > 255) {
            y = 255;
        }

        int x = 0;
        for(int i = 0; i < num_leds; ++i) {
            set_pixel(lerp(c[x], c[x + 1], y), i);
            if(++x == 6) {
                x = 0;
            }
        }
    }

    color c[6];
};

//////////////////////////////////////////////////////////////////////

struct solid_effect : effect
{
    void init()
    {
        for(int i = 0; i < num_leds; ++i) {
            c[i] = get_color_parameter(i % 6);
        }
    }

    void update()
    {
        for(int i = 0; i < num_leds; ++i) {
            set_pixel(c[i], i);
        }
    }
    color c[num_leds];
};

//////////////////////////////////////////////////////////////////////

struct sparkle_effect : effect
{
    void init()
    {
        Random r(frames);
        for(int i = 0; i < 6; ++i) {
            c[i] = get_color_parameter(i);
        }
        for(int i = 0; i < num_leds; ++i) {
            id[i] = rand() % 6;
            int x = i % 6;
            int y = (x + 1) % 6;
            w[i].init(c[x], c[y]);
            w[i].vel = value::speed(speed_ranges[0]) * ((r.next_byte() & 2) - 1) >> 2;
            w[i].val = r.next_byte();
        }
    }

    // every N frames,
    void update()
    {
        Random r(frames);
        int x;
        for(int i = 0; i < num_leds; ++i) {
            switch(w[i].update(speed_ranges[0])) {
            case 0:
                break;
            case value::hit_bottom:
                x = id[i] % 6;
                w[i].c[1] = c[x];
                c[x] = get_color_parameter(x);    // in case the color is random
                break;
            case value::hit_top:
                id[i] += 1;
                w[i].c[0] = c[(id[i] + 1) % 6];
                break;
            }
            set_pixel(w[i].get_color(), i);
        }
    }

    value w[num_leds];
    byte id[num_leds];
    color c[6];
};

//////////////////////////////////////////////////////////////////////

struct flash_effect : effect
{
    void init()
    {
        speed = 120 - ((global_speed * 109) >> 8);
        seed = frames;
        for(int i = 0; i < 6; ++i) {
            c[i] = get_color_parameter(i);
        }
        do_frame();
    }

    // every N frames,
    void update()
    {
        if((frames % speed) == 0) {
            init();
        } else {
            do_frame();
        }
    }

    void do_frame()
    {
        Random rnd(seed);
        for(int i = 0; i < num_leds; ++i) {
            set_pixel(c[rnd.next() % 6], i);
        }
    }
    int seed;
    int speed;
    color c[6];
};

//////////////////////////////////////////////////////////////////////

struct twinkle_effect : effect
{
    void init()
    {
        Random r(frames);
        color c[6];
        for(int i = 0; i < 6; ++i) {
            c[i] = get_color_parameter(i);
        }
        for(int i = 0; i < num_leds; ++i) {
            int x = r.next() % 6;
            w[i].init(c[x], c[(x + 1) % 6]);
            w[i].vel = value::speed(speed_ranges[0]);
            w[i].val = r.next() & 255;
        }
    }

    // every N frames,
    void update()
    {
        Random r(frames);
        color c[6];
        for(int i = 0; i < 6; ++i) {
            c[i] = get_color_parameter(i);
        }
        for(int i = 0; i < num_leds; ++i) {
            switch(w[i].update(speed_ranges[0])) {
            case 0:
                break;
            case value::hit_bottom:
                break;
            case value::hit_top:
                int x = r.next() % 6;
                w[i].init(c[x], c[(x + 1) % 6]);
                w[i].val = 0;
                w[i].vel = value::speed(speed_ranges[0]);
                break;
            }
            set_pixel(w[i].get_color(), i);
        }
    }
    value w[num_leds];
};

//////////////////////////////////////////////////////////////////////

struct glow_effect : effect
{
    void init()
    {
        x = 0;
        for(int i = 0; i < 6; ++i) {
            c[i] = get_color_parameter(i);
        }
    }

    // every N frames,
    void update() override
    {
        x += global_speed >> 3;
        if(x >= (2048 * 3)) {
            init();
        }
        uint a = (x + 512) / 1024;
        uint p = ((a + 1) & 0xfe) % 6;
        uint q = ((a & 0xfe) + 1) % 6;
        int b = ease_in_out((sin(x) + 255) >> 1);
        color r = lerp(c[p], c[q], b);
        for(int i = 0; i < num_leds; ++i) {
            set_pixel(r, i);
        }
    }

    color c[6];
    int x;
};

//////////////////////////////////////////////////////////////////////
// template nonsense to get sizeof(largest effect)

template <typename... Ts> struct largest_type;

template <typename T> struct largest_type<T>
{
    using type = T;
};

template <typename T, typename U, typename... Ts> struct largest_type<T, U, Ts...>
{
    using type = typename largest_type<typename std::conditional<(sizeof(U) <= sizeof(T)), T, U>::type, Ts...>::type;
};

//////////////////////////////////////////////////////////////////////
// use that to get how big the buffer for holding the largest effect needs to be

#define EFFECTOR(x) x##_effect,

using largest_t = typename largest_type<
#include "effect_list.h"
    byte>::type;

// declare buffer for holding the active effect
byte effect_object_buffer[sizeof(largest_t)];

//////////////////////////////////////////////////////////////////////
// create vtable initializer for each one

#undef EFFECTOR
#define EFFECTOR(x)                                     \
    void x##_effect_create()                            \
    {                                                   \
        ESP_LOGI(TAG, "Create " #x " effect");          \
        new((void *)effect_object_buffer) x##_effect(); \
    }

#include "effect_list.h"

//////////////////////////////////////////////////////////////////////
// create list of fn pointers to the creators

#undef EFFECTOR
#define EFFECTOR(x) x##_effect_create,

typedef void(effect_creator)();

effect_creator *descriptors[] = {
#include "effect_list.h"
    null
};

//////////////////////////////////////////////////////////////////////

void effect::frame_update(byte *frame_buffer)
{
    // backbuffer
    rgb_buffer = reinterpret_cast<color *>(frame_buffer);

    // if new alexa command arrived
    settings_t settings;
    if(xQueueReceive(settings_queue, &settings, 0)) {

        // init the vtable
        descriptors[settings.effect]();
        global_effect = (effect *)effect_object_buffer;

        // 6 indices because 6 divisible by 1, 2, 3 (possible # of colors)
        int index[3][6] = { { 0, -1, 0, -1, 0, -1 }, { 0, 1, 0, 1, 0, 1 }, { 0, 1, 2, 0, 1, 2 } };

        // sometimes, replace single color odd indices with 0 (eg cycle against color 0, not 'off')

        if((settings.effect == settings_t::solid_effect && settings.num_colors == 1) ||
           ((settings.effect == settings_t::sparkle_effect || settings.effect == settings_t::flash_effect || settings.effect == settings_t::glow_effect) &&
            settings.num_colors == 1 && settings.color[0] == color::random_color)) {

            index[0][1] = 0;
            index[0][3] = 0;
            index[0][5] = 0;
        }

        // setup color ranges for 1..3 colors
        for(int i = 0; i < 6; ++i) {
            uint32 color_low;
            uint32 color_high;
            int idx = index[settings.num_colors - 1][i];
            if(idx < 0) {
                color_low = 0;    // flashing a single color with nothing
                color_high = 0;
            } else {
                color_low = settings.color[idx];
                color_high = settings.color[idx];
            }
            if(color_low == color::random_color) {
                color_low = 0;
                color_high = 0xffffff;
            }
            global_effect->color_ranges[i].low.set(color_low);
            global_effect->color_ranges[i].high.set(color_high);
        }

        // speed not really handled, just offset it
        for(int i = 0; i < max_speed_parameters; ++i) {
            global_effect->speed_ranges[i].low = settings.speed + 24;
            global_effect->speed_ranges[i].high = settings.speed + 48;
        }

        // reset effect time when a new one is requested
        frames = 0;

        // handle brightness/powerstate
        color black = 0;
        color white(255, 220, 235);
        global_dimmer = lerp(black, white, settings.brightness * 255 / 100);
        bool powerstate = (settings.flags & (uint32)settings_t::flags_t::powerstate) != 0;
        powerswitch_set(powerstate);

        ESP_LOGI(TAG, "Setting powerstate to %d", powerstate);

        // finally, let the effect init() itself
        global_effect->init();
    }

    // and render the frame
    frames += 1;
    if(global_effect != null) {

        // render effect into frame buffer
        global_effect->update();

        // scale the brightness to clamp the power usage

        // get total of the 3 channels
        int r = 0, g = 0, b = 0;
        byte *p = frame_buffer;
        for(int i = 0; i < num_leds; ++i) {
            r += *p++;
            g += *p++;
            b += *p++;
        }

        // measured power consumption of the 3 led colors in mA * 100 (red = 2.1A)
        int red_amps = 21;
        int green_amps = 16;
        int blue_amps = 18;

        r *= red_amps;
        g *= green_amps;
        b *= blue_amps;

        // total power consumption for this frame
        int sum = r + g + b;

        int scale = 256;

        // exceeded limit? scale it down to max_power (which is 85% of full load = ~4A)

        int string = num_leds * 255;
        int abs_max_power = (string * red_amps) + (string * green_amps) + (string * blue_amps);
        int max_power = (abs_max_power * 75) / 100;

        if(sum > max_power) {

            scale = max_power * 256 / sum;
            if(scale > 256) {
                scale = 256;
            }
        }

        if(frames == 60) {
            ESP_LOGI(TAG, "Power scale: %d", scale);
        }

        int num_bytes = num_leds * 3;
        p = frame_buffer;
        for(int i = 0; i < num_bytes; ++i) {
            int b = *p;
            *p++ = (b * scale) >> 8;
        }
    }
}

//////////////////////////////////////////////////////////////////////
