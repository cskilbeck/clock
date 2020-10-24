//////////////////////////////////////////////////////////////////////
// RGB and HSV color admin

#include <cstdint>
#include <cstddef>
#include "util.h"
#include "random.h"
#include "color.h"

//////////////////////////////////////////////////////////////////////

void color::randomize()
{
    r = rand8();
    g = rand8();
    b = rand8();
}

//////////////////////////////////////////////////////////////////////

void color::from_hsv(byte h, byte s, byte v)
{
    if(s == 0) {
        r = g = b = v;
        return;
    }

    // int region = h / 43;
    constexpr int inv_43 = 65536 / 43;
    int region = (h * inv_43) >> 16;

    int remainder = (h - (region * 43)) * 6;

    int p = (v * (255 - s)) >> 8;
    int q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    int t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch(region) {
    case 0:
        r = v;
        g = t;
        b = p;
        break;
    case 1:
        r = q;
        g = v;
        b = p;
        break;
    case 2:
        r = p;
        g = v;
        b = t;
        break;
    case 3:
        r = p;
        g = q;
        b = v;
        break;
    case 4:
        r = t;
        g = p;
        b = v;
        break;
    default:
        r = v;
        g = p;
        b = q;
        break;
    }
}

//////////////////////////////////////////////////////////////////////

color color::rgb_from_hsv()
{
    color c;
    c.from_hsv(r, g, b);
    return c;
}

//////////////////////////////////////////////////////////////////////

color color::hsv_from_rgb()
{
    color hsv;

    byte mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
    byte mx = r > g ? (r > b ? r : b) : (g > b ? g : b);

    hsv.v = mx;
    if(hsv.v == 0) {
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * int32(mx - mn) / hsv.v;
    if(hsv.s == 0) {
        hsv.h = 0;
        return hsv;
    }

    int mm = mx - mn;
    if(mx == r) {
        hsv.h = 0 + 43 * (g - b) / mm;
    } else if(mx == g) {
        hsv.h = 85 + 43 * (b - r) / mm;
    } else {
        hsv.h = 171 + 43 * (r - g) / mm;
    }

    return hsv;
}

//////////////////////////////////////////////////////////////////////

color color::random()
{
    color c;
    c.randomize();
    return c;
}
