#pragma once

//////////////////////////////////////////////////////////////////////

#pragma pack(push, 1)
struct color_range
{
    color low;
    color high;

    color get() const
    {
        int r = rand() % (high.r - low.r + 1) + low.r;
        int g = rand() % (high.g - low.g + 1) + low.g;
        int b = rand() % (high.b - low.b + 1) + low.b;
        return color(r, g, b);
    }
};
#pragma pack(pop)

//////////////////////////////////////////////////////////////////////

#pragma pack(push, 1)
struct speed_range
{
    int16 low;
    int16 high;

    int16 get() const
    {
        return rand() % (high - low + 1) + low;
    }

    int16 mid() const
    {
        return (high + low) / 2;
    }
};
#pragma pack(pop)

//////////////////////////////////////////////////////////////////////

struct effect
{
    static int const max_color_parameters = 8;
    static int const max_speed_parameters = 8;

    color_range color_ranges[max_color_parameters];
    speed_range speed_ranges[max_speed_parameters];

    void cls(color c);

    color get_color_parameter(int index);
    byte get_speed_parameter(int index);

    void set_pixel(color c, int led);
    void set_pixels(color c, int led, int n, int stride = 1);

    virtual void init();
    virtual void update();

    static void frame_update(byte *frame_buffer);

    static effect *global_effect;
    static color *rgb_buffer;
};

//////////////////////////////////////////////////////////////////////
