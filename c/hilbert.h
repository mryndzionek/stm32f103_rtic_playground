#ifndef _HILBERT_H_
#define _HILBERT_H_

#include <stdint.h>

#define NUM_TAPS (31)

typedef int16_t q15_t;

typedef struct
{
    q15_t xs[NUM_TAPS];
    size_t i;
    size_t j;
} hilb_t;

void hilb_init(hilb_t *f);
void hilb_update(hilb_t *f, q15_t x, q15_t *yi, q15_t *yq);
q15_t hilb_get_amp(q15_t a, q15_t b);

#endif // _HILBERT_H
