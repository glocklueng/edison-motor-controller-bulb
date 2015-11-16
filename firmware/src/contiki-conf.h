#ifndef _CONTIKI_CONF_H_
#define _CONTIKI_CONF_H_

#include <stdint.h>

#define CCIF
#define CLIF

#define WITH_UIP   0
#define WITH_ASCII 1

#define CLOCK_CONF_SECOND 1000

/* These names are deprecated, use C99 names. */
typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int8_t s8_t;
typedef int16_t s16_t;
typedef int32_t s32_t;

typedef unsigned int clock_time_t;
typedef unsigned int uip_stats_t;

#ifndef BV
#define BV(x) (1<<(x))
#endif

#define UIP_CONF_UDP     1
#define UIP_CONF_TCP     1
#define UIP_CONF_LLH_LEN 14

#define CC_BYTE_ALIGNED __attribute__ ((packed, aligned(1)))

#define RAND_MAX 0x7fff

#endif // _CONTIKI_CONF_H_
