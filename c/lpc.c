#include "lpc.h"

#include <stdio.h>
#include <assert.h>

#include "fix.h"

#define DC_BLOCK_FACTOR (0xF3333)

struct _lpc_filter_t
{
    size_t i;
    size_t t;
    fix16_t g;
    uint8_t ps;
    fix16_t v[LPC_ORDER];
    fix16_t a[LPC_ORDER];
    fix16_t de_y;
    fix16_t dc_y;
};

struct _lpc_seq_decoder_t
{
    lpc_filter_t *f;
    size_t n;
    size_t i;
    size_t j;
    bool finished;
    lpc_seq_t const *s[MAX_DECODER_SEQ];
};

static fix16_t excitation(lpc_filter_t *f, uint32_t rnd)
{
    static size_t last_p = 0; 
    fix16_t y;

    if (f->ps == 0)
    {
        // unvoiced
        fix16_t noise = (rnd % (2 * FIX_ONE)) - FIX_ONE;
        y = fix_mul(f->g, noise);
    }
    else
    {
        // voiced
        if ((f->t - last_p) >= f->ps)
        {
            last_p = f->t;
            y = f->g;
        }
        else
        {
            fix16_t noise = (rnd % (2 * FIX_ONE)) - FIX_ONE;
            y = fix_mul(0.1, noise);
        }
    }
    return y;
}

lpc_filter_t *lpc_filter_new(void)
{
    static size_t i = 0;
    static lpc_filter_t fs[MAX_LPC_OBJECTS];

    if (i < MAX_LPC_OBJECTS)
    {
        return &fs[i++];
    }
    else
    {
        return NULL;
    }
}

void lpc_filter_reset(lpc_filter_t *f)
{
    for (size_t i = 0; i < LPC_ORDER; i++)
    {
        f->v[i] = FIX_ZERO;
        f->a[i] = FIX_ZERO;
    }

    f->de_y = FIX_ZERO;
    f->dc_y = FIX_ZERO;
    f->g = FIX_ZERO;
    f->i = 0;
    f->ps = 0;
    f->t = 0;
}

void lpc_filter_update(lpc_filter_t *f, const int16_t a[LPC_ORDER], int16_t g, uint8_t ps)
{
    f->g = i16_to_fix(g);
    f->ps = ps;

    for (size_t i = 0; i < LPC_ORDER; i++)
    {
        f->a[i] = i16_to_fix(a[i]);
    }
}

fix16_t lpc_filter_exec(lpc_filter_t *f, uint32_t rnd)
{
    fix16_t y;
    fix16_t ny = excitation(f, rnd);

    for (size_t k = 0; k < LPC_ORDER; k++)
    {
        ny = fix_add(ny, fix_mul(f->a[k], f->v[k]));
    }

    // de-emphasis
    fix16_t de_y = fix_add(ny, fix_mul(LPC_DEEMPHASIS_FACTOR, f->de_y));

    // dc-blocking
    y = fix_add(fix_add(de_y, -f->de_y), fix_mul(DC_BLOCK_FACTOR, f->dc_y));
    f->de_y = de_y;
    f->dc_y = y;

    for (size_t i = LPC_ORDER - 1; i > 0; i--)
    {
        f->v[i] = f->v[i - 1];
    }

    f->v[0] = ny;
    f->t++;
    f->i++;

    if (f->i == LPC_FRAME_LEN)
    {
        f->i = 0;
    }

    return y;
}

lpc_seq_decoder_t *lpc_seq_decoder_new(void)
{
    static size_t i = 0;
    static lpc_seq_decoder_t ds[MAX_LPC_OBJECTS];

    if (i < MAX_LPC_OBJECTS)
    {
        lpc_seq_decoder_t *dec = &ds[0];
        dec->f = lpc_filter_new();
        if (dec->f)
        {
            i++;
            return dec;
        }
        else
        {
            return NULL;
        }
    }
    else
    {
        return NULL;
    }
}

size_t lpc_seq_decoder_update(lpc_seq_decoder_t *self, lpc_seq_t const *const *s, size_t n)
{
    size_t size = 0;
    assert(n <= MAX_DECODER_SEQ);

    for (size_t i = 0; i < n; i++)
    {
        self->s[i] = s[i];
        size += s[i]->len * LPC_FRAME_LEN;
    }

    self->n = n;
    self->i = 0;
    self->j = 0;
    self->finished = false;
    lpc_filter_reset(self->f);

    return size;
}

bool lpc_seq_decoder_exec(lpc_seq_decoder_t *self, uint32_t rnd, fix16_t *y)
{
    if (self->finished)
    {
        return self->finished;
    }

    if (self->f->i == 0)
    {
        if (self->j == self->s[self->i]->len)
        {
            if (++self->i == self->n)
            {
                self->finished = true;
                return true;
            }
            self->j = 0;
        }
        lpc_seq_t const *s = self->s[self->i];
        lpc_filter_update(self->f,
                          s->frames[self->j].a,
                          s->frames[self->j].g,
                          s->frames[self->j].ps);
        self->j++;
    }

    *y = lpc_filter_exec(self->f, rnd);

    return false;
}
