
#ifndef _PLATFORM_CONFIG_H_
#define _PLATFORM_CONFIG_H_

#include <pinout.h>

#define DEBUG_UART   huart3
#define SPI          hspi1
#define MAX_TIMEOUT  0xffff

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
//extern IWDG_HandleTypeDef hiwdg;
extern SPI_HandleTypeDef hspi1;

#endif
