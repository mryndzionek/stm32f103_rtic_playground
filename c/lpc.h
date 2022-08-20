#ifndef __LPC__
#define __LPC__

#include <stdbool.h>

#include "fix.h"
#include "lpc_data.h"

#define MAX_LPC_OBJECTS (2)
#define MAX_DECODER_SEQ (5)

typedef struct _lpc_filter_t lpc_filter_t;
typedef struct _lpc_seq_decoder_t lpc_seq_decoder_t;

lpc_filter_t *lpc_filter_new(void);
void lpc_filter_update(lpc_filter_t *f, const int16_t a[LPC_ORDER], int16_t g, uint8_t ps);
void lpc_filter_reset(lpc_filter_t *self);
fix16_t lpc_filter_exec(lpc_filter_t *f, uint32_t rnd);

lpc_seq_decoder_t *lpc_seq_decoder_new(void);
size_t lpc_seq_decoder_update(lpc_seq_decoder_t *self, lpc_seq_t const *const *s, size_t n);
bool lpc_seq_decoder_exec(lpc_seq_decoder_t *self, uint32_t rnd, fix16_t *y);

#endif