#ifndef __FIX_H__
#define __FIX_H__

#include <stdint.h>
#include <stdlib.h>

#define FIX_ONE (0x00100000)
#define FIX_ZERO (0x00000000)

typedef int32_t fix16_t;

static inline float fix_to_float(fix16_t a) { return (float)a / FIX_ONE; }
static inline fix16_t float_to_fix(float a)
{
    float temp = a * FIX_ONE;
    return (fix16_t)temp;
}

static inline fix16_t fix_add(fix16_t x, fix16_t y) { return x + y; }

static inline fix16_t fix_mul(fix16_t x, fix16_t y)
{
    int64_t res = (int64_t)x * y;
    return (fix16_t)(res >> 20);
}

static inline fix16_t i16_to_fix(int16_t x)
{
    if (x >= 0)
    {
        return x << 8;
    }
    else
    {
        return -(abs(x) << 8);
    }
}

#endif