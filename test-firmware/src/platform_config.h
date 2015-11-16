
#ifndef _PLATFORM_CONFIG_H_
#define _PLATFORM_CONFIG_H_

#include <pinout.h>

#define MAX_TIMEOUT  0xffff
#define DEBUG_UART   huart1
#define SPI          hspi1

extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;

#endif
