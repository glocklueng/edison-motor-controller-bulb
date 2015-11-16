#DEBUGFLAGS      = -Os
#USER_LDFLAGS    = --specs=nano.specs
USER_CFLAGS     = -Ilibs/contiki/core -Ilibs/contiki/core/net/
#USE_FULL_ASSERT =
SRCS = \
	src/edison-motor-controller-bulb.c \
	libs/contiki/core/sys/process.c \
	libs/contiki/core/sys/etimer.c \
	libs/contiki/core/sys/timer.c
SSRCS =
