USER_CFLAGS     = -Ilibs/contiki/core -Ilibs/contiki/core/net/
SRCS = \
	src/stm32-tester.c \
	libs/utils/debug.c \
	libs/utils/ringbufferdma.c \
	libs/utils/syscalls.c \
	libs/utils/time.c \
	libs/utils/utils.c \
	libs/slre/slre.c
SSRCS =
