#pragma once

//////////////////////////////////////////////////////////////////////

uint16 IRAM_ATTR crc16(byte const *p, size_t len);
uint16 IRAM_ATTR crc16_copy(byte const *p, byte *dst, size_t len);

//////////////////////////////////////////////////////////////////////

template <typename T, size_t N> constexpr size_t countof(T const (&)[N]) noexcept
{
    return N;
}

//////////////////////////////////////////////////////////////////////

template <typename T> T min(T const &a, T const &b)
{
    return (a < b) ? a : b;
}

//////////////////////////////////////////////////////////////////////

template <typename T> T max(T const &a, T const &b)
{
    return (a > b) ? a : b;
}

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

//////////////////////////////////////////////////////////////////////

namespace detail
{
    template <typename F> class defer_finalizer
    {
        F f;
        bool moved;

    public:
        template <typename T> defer_finalizer(T &&f_) : f(std::forward<T>(f_)), moved(false)
        {
        }

        defer_finalizer(const defer_finalizer &) = delete;

        defer_finalizer(defer_finalizer &&other) : f(std::move(other.f)), moved(other.moved)
        {
            other.moved = true;
        }

        ~defer_finalizer()
        {
            if(!moved) {
                f();
            }
        }
    };

    static struct
    {
        template <typename F> defer_finalizer<F> operator<<(F &&f)
        {
            return defer_finalizer<F>(std::forward<F>(f));
        }
    } deferrer;

}    // namespace detail

//////////////////////////////////////////////////////////////////////

#define DEFER_TOKENPASTE(x, y) x##y
#define DEFER_TOKENPASTE2(x, y) DEFER_TOKENPASTE(x, y)
#define scoped auto DEFER_TOKENPASTE2(__deferred_lambda_call, __COUNTER__) = detail::deferrer <<
#define defer(X) \
    scoped[=]    \
    {            \
        X;       \
    };
