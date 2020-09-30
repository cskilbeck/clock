#pragma once

#if !defined(__cplusplus)
#error "util.h is a C++ header - can't include this from a C file"
#endif

using uint = unsigned int;
using int64 = int64_t;
using int32 = int32_t;
using int16 = int16_t;
using int8 = int8_t;
using uint64 = uint64_t;
using uint32 = uint32_t;
using uint16 = uint16_t;
using uint8 = uint8_t;
using byte = uint8_t;

constexpr auto null = nullptr;

namespace util
{

// workaround to allow shift operator within template declaration
template <uint32 N, int S> struct shr
{
    enum
    {
        v = N >> S
    };
};

template <uint32 N> struct hi_bit
{
    enum
    {
        value = hi_bit<shr<N, 1>::v>::value + 1
    };
};

template <> struct hi_bit<0>
{
    enum
    {
        value = -1
    };
};

}

// position of high bit in a uint32 or -1 if none are set
#define PIN_POS(X) (util::hi_bit<X>::value)
