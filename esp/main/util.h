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

constexpr nullptr_t null = nullptr;

//////////////////////////////////////////////////////////////////////

template <typename T, size_t N> constexpr size_t countof(T const (&)[N]) noexcept
{
    return N;
}

//////////////////////////////////////////////////////////////////////

int sin(int x);    // pi = 1024, amplitude = +255 .. -255
int ease_in_out(int x);

// ARGH, get this somewhere else

constexpr int num_leds = 400;

//////////////////////////////////////////////////////////////////////

template <typename T> static T lerp(T const &s, T const &d, int l)
{
    int r = (((d.r - s.r) * l) >> 8) + s.r;
    int g = (((d.g - s.g) * l) >> 8) + s.g;
    int b = (((d.b - s.b) * l) >> 8) + s.b;
    return T(r, g, b);
}
