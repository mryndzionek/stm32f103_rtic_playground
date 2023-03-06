#include <stdio.h>
#include <assert.h>

#include "hilbert.h"

static const q15_t h_taps[NUM_TAPS] = {20, 0, 44, 0, 185, -1, 534, 1, 1272, 2, 2736, 4, 5990, 3, 20522, 0,
                                       -20522, -3, -5990, -4, -2736, -2, -1272, -1, -534, 1, -185, 0, -44, 0, -20};

static q15_t q15_add(q15_t a, q15_t b)
{
    int32_t c = (uint32_t)a + (uint32_t)b;

    if (c > 32767)
        c = 32767;
    else if (c < -32768)
        c = -32768;

    return (q15_t)c;
}

static q15_t q15_mul(q15_t a, q15_t b)
{
    int32_t c = ((int32_t)a * (int32_t)b) >> 15;
    return (q15_t)c;
}

static q15_t q15_sqrt(q15_t v)
{
    uint32_t t, q, b, r;
    r = v;
    b = 0x8000;
    q = 0;

    while (b > 0x4)
    {
        t = q + b;
        if (r >= t)
        {
            r -= t;
            q = t + b; // equivalent to q += 2*b
        }
        r <<= 1;
        b >>= 1;
    }
    q >>= 1;

    return q;
}

void hilb_init(hilb_t *f)
{
    for (size_t i = 0; i < NUM_TAPS; i++)
    {
        f->xs[i] = 0;
    }
    f->i = 0;
}

void hilb_update(hilb_t *f, q15_t x, q15_t *yi, q15_t *yq)
{
    q15_t y = 0;

    f->xs[f->i] = x;
    f->i++;
    if (f->i >= NUM_TAPS)
    {
        f->i = 0;
    }

    for (size_t i = 0; i < NUM_TAPS; i++)
    {
        size_t j = (f->i + i) % NUM_TAPS;
        y = q15_add(y, q15_mul(h_taps[i], f->xs[j]));
    }

    *yq = y;
    *yi = f->xs[(f->i + ((NUM_TAPS - 1) >> 1)) % NUM_TAPS];
}

q15_t hilb_get_amp(q15_t a, q15_t b)
{
    return q15_sqrt(q15_add(q15_mul(a, a), q15_mul(b, b)));
}
