
#ifndef _PLATFORM_CONFIG_H_
#define _PLATFORM_CONFIG_H_

#include <pinout.h>

#define DEBUG_UART       huart2
#define SPI              hspi1
#define MAX_TIMEOUT      0xffff
#define COMPASS_TIMER_MS 1000

#define MOTOR_LEFT_PWM_HANDLE      (&htim2)
#define MOTOR_LEFT_PWM_CHANNEL     TIM_CHANNEL_1
#define MOTOR_LEFT_PWM_IT_CHANNEL  HAL_TIM_ACTIVE_CHANNEL_1
#define MOTOR_RIGHT_PWM_HANDLE     (&htim2)
#define MOTOR_RIGHT_PWM_CHANNEL    TIM_CHANNEL_2
#define MOTOR_RIGHT_PWM_IT_CHANNEL HAL_TIM_ACTIVE_CHANNEL_2

extern UART_HandleTypeDef huart2;
//extern IWDG_HandleTypeDef hiwdg;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;

#endif
