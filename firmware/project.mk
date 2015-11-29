#DEBUGFLAGS      = -Os
#USER_LDFLAGS    = --specs=nano.specs
USER_CFLAGS     = -DCONTIKI -Ilibs/contiki/core -Ilibs/contiki/core/net/
#USER_CFLAGS     += -DDEBUG_LIS3MDL 
USE_FULL_ASSERT =
SRCS = \
	src/edison-motor-controller-bulb.c \
	libs/contiki/core/sys/process.c \
	libs/contiki/core/sys/etimer.c \
	libs/contiki/core/sys/timer.c \
	libs/utils/debug.c \
	libs/utils/ringbufferdma.c \
	libs/utils/syscalls.c \
	libs/utils/time.c \
	libs/utils/utils.c \
	libs/utils/trig_int16.c \
	libs/lis3mdl/lis3mdl.c
SSRCS =
