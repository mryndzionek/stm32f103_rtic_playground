#ifndef __LPC_DATA__
#define __LPC_DATA__

#include <stddef.h>

#include "fix.h"

#define LPC_ORDER (8)
#define LPC_FRAME_LEN (440)
#define LPC_SAMPLE_RATE (11000)
#define LPC_DEEMPHASIS_FACTOR (0xF8CAC)

typedef struct
{
    int16_t g;
    uint8_t ps;
    int16_t a[LPC_ORDER];
} lpc_frame_t;

typedef struct
{
    const size_t len;
    lpc_frame_t frames[];
} lpc_seq_t;

typedef enum {
    LPC_ZEROWA = 0,
    LPC_PIERWSZA,
    LPC_DRUGA,
    LPC_TRZECIA,
    LPC_CZWARTA,
    LPC_PIATA,
    LPC_SZOSTA,
    LPC_SIODMA,
    LPC_OSMA,
    LPC_DZIEWIATA,
    LPC_DZIESIATA,
    LPC_JEDENASTA,
    LPC_DWUNASTA,
    LPC_TRZYNASTA,
    LPC_CZTERNASTA,
    LPC_PIETNASTA,
    LPC_SZESNASTA,
    LPC_SIEDEMNASTA,
    LPC_OSIEMNASTA,
    LPC_DZIEWIETNASTA,
    LPC_DWUDZIESTA,
    LPC_ZERO,
    LPC_JEDEN,
    LPC_DWIE,
    LPC_TRZY,
    LPC_CZTERY,
    LPC_PIEC,
    LPC_SZESC,
    LPC_SIEDEM,
    LPC_OSIEM,
    LPC_DZIEWIEC,
    LPC_DZIESIEC,
    LPC_JEDENASCIE,
    LPC_DWANASCIE,
    LPC_TRZYNASCIE,
    LPC_CZTERNASCIE,
    LPC_PIETNASCIE,
    LPC_SZESNASCIE,
    LPC_SIEDEMNASCIE,
    LPC_OSIEMNASCIE,
    LPC_DZIEWIETNASCIE,
    LPC_DWADZIESCIA,
    LPC_TRZYDZIESCI,
    LPC_CZTERDZIESCI,
    LPC_PIECDZIESIAT,
    LPC_JEST_GODZINA,
    LPC_MAX_SEQ
} lpc_seq_e;

const lpc_seq_t * const lpc_get_seq(lpc_seq_e id);

#endif
