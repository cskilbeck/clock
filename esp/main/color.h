#pragma once

//////////////////////////////////////////////////////////////////////

#pragma pack(push, 1)
struct color
{
    union
    {
        struct
        {
            byte r, g, b;
        };
        struct
        {
            byte h, s, v;
        };
    };

    color() : r(0), g(0), b(0)
    {
    }

    color(byte red, byte green, byte blue) : r(red), g(green), b(blue)
    {
    }

    color(uint32 c)
    {
        set(c);
    }

    void from_hsv(byte h, byte s, byte v);

    void set(uint32 c)
    {
        r = (c >> 16) & 0xff;
        g = (c >> 8) & 0xff;
        b = c & 0xff;
    }

    void randomize();

    color rgb_from_hsv();
    color hsv_from_rgb();

    static color random();

    // this special uint32 value means color should be randomized
    enum
    {
        random_color = 0x80000000,
        red = 0xff0000,
        green = 0x00ff00,
        blue = 0x0000ff
    };
};
#pragma pack(pop)

//////////////////////////////////////////////////////////////////////
