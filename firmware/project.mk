#DEBUGFLAGS      = -Os
#USER_LDFLAGS    = --specs=nano.specs
USER_CFLAGS     = -DCONTIKI -Ilibs/contiki/core -Ilibs/contiki/core/net/
#USER_CFLAGS     += -DDEBUG_LIS3MDL 
FEATURES        += FLASH
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
	libs/utils/math.c \
	libs/lis3mdl/lis3mdl.c
SSRCS =
LINK_FLASH_START       = 0x08003000
FLASH = 53248
LINK_DATA_EEPROM_START = 0x08080080
