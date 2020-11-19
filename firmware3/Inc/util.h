#pragma once

#if !defined(__cplusplus)
#error "util.h is a C++ header - can't include this from a C file"
#else

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
    template <uint32 N> struct shr
    {
        enum
        {
            v = N >> 1
        };
    };

    template <uint32 N> struct hi_bit
    {
        enum
        {
            position = hi_bit<shr<N>::v>::position + 1
        };
    };

    template <> struct hi_bit<0>
    {
        enum
        {
            position = -1
        };
    };

    template <typename T> T abs(T a)
    {
        return (a < 0) ? -a : a;
    }

    template <typename T> T max(T a, T b)
    {
        return (a > b) ? a : b;
    }

    template <typename T> T min(T a, T b)
    {
        return (a < b) ? a : b;
    }
    
    template <typename T, std::size_t N> constexpr std::size_t countof(T const (&)[N]) noexcept
    {
        return N;
    }

}    // namespace util

//////////////////////////////////////////////////////////////////////
// get the largest type from a list of types

template <typename... Ts> struct largest_type;

template <typename T> struct largest_type<T>
{
    using type = T;
};

template <typename T, typename U, typename... Ts> struct largest_type<T, U, Ts...>
{
    using type = typename largest_type<typename std::conditional<(sizeof(U) <= sizeof(T)), T, U>::type, Ts...>::type;
};

// position of high bit in a uint32 or -1 if none are set
#define PIN_POS(X) (util::hi_bit<X>::position)

#endif