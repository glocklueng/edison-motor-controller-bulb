
#ifndef _PLATFORM_CONFIG_H_
#define _PLATFORM_CONFIG_H_

#include <pinout.h>

#define DEBUG_UART       huart3
#define SPI              hspi1
#define MAX_TIMEOUT      0xffff
#define COMPASS_TIMER_MS 10

#define MOTOR_LEFT_PWM_HANDLE    (&htim1)
#define MOTOR_LEFT_PWM_CHANNEL   TIM_CHANNEL_1
#define MOTOR_RIGHT_PWM_HANDLE   (&htim1)
#define MOTOR_RIGHT_PWM_CHANNEL  TIM_CHANNEL_3

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
//extern IWDG_HandleTypeDef hiwdg;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;

#endif
