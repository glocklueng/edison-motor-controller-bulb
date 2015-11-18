
#include "platform_config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <utils/debug.h>
#include <utils/time.h>
#include <slre/slre.h>
#include "edison-motor-controller-bulb.h"

void setup() {
  printf("begin setup\n");
  debug_setup();
  
  GPIO_InitTypeDef gpioInit;
  gpioInit.Pull = GPIO_NOPULL;
  gpioInit.Speed = GPIO_SPEED_HIGH;
  gpioInit.Pin = GPIO_PIN_9;
  gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOC, &gpioInit);
  
  printf("setup complete\n");
  printf("> ");
}

void loop() {
  debug_tick();
}

void debug_processLine(const char* line) {
  uint8_t cmd;
  EdisonSocketConfig config;
  EdisonMotorCommandDrive drive;
  EdisonMotorCommandStatusResponse status;
  struct slre_cap caps[4];

  if (strlen(line) == 0) {
  } else if(strcmp(line, "c") == 0) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    sleep_ms(100);
    cmd = EDISON_SOCKET_CMD_READ_CONFIG;
    HAL_SPI_Transmit(&SPI, &cmd, 1, MAX_TIMEOUT);
    sleep_ms(100);
    HAL_SPI_Receive(&SPI, (uint8_t*)&config, sizeof(config), MAX_TIMEOUT);
    printf("version: 0x%02x\n", config.version);
    printf("url: %.*s\n", sizeof(config.driverUrl), config.driverUrl);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  } else if(strcmp(line, "s") == 0) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    sleep_ms(100);
    cmd = EDISON_MOTOR_CMD_STATUS;
    HAL_SPI_Transmit(&SPI, &cmd, 1, MAX_TIMEOUT);
    sleep_ms(100);
    HAL_SPI_Receive(&SPI, (uint8_t*)&status, sizeof(status), MAX_TIMEOUT);
    printf("heading: %d\n", status.heading);
    printf("targetHeading: %d\n", status.targetHeading);
    printf("speedLeft: %d\n", status.speedLeft);
    printf("distanceLeft: %d\n", status.distanceLeft);
    printf("speedRight: %d\n", status.speedRight);
    printf("distanceRight: %d\n", status.distanceRight);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  } else if(slre_match("d(.+),(.+)", line, strlen(line), caps, 2, 0) > 0) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    sleep_ms(100);
    cmd = EDISON_MOTOR_CMD_DRIVE;
    HAL_SPI_Transmit(&SPI, &cmd, 1, MAX_TIMEOUT);
    sleep_ms(100);
    drive.distanceLeft = EDISON_MOTOR_DISTANCE_NOT_SET;
    drive.speedLeft = strtol(caps[0].ptr, NULL, 10);
    drive.distanceRight = EDISON_MOTOR_DISTANCE_NOT_SET;
    drive.speedRight = strtol(caps[1].ptr, NULL, 10);
    drive.targetHeading = EDISON_MOTOR_TARGET_HEADING_NOT_SET;
    printf("targetHeading: %d\n", drive.targetHeading);
    printf("speedLeft: %d\n", drive.speedLeft);
    printf("distanceLeft: %d\n", drive.distanceLeft);
    printf("speedRight: %d\n", drive.speedRight);
    printf("distanceRight: %d\n", drive.distanceRight);
    HAL_SPI_Transmit(&SPI, (uint8_t*)&drive, sizeof(drive), MAX_TIMEOUT);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  } else {
    printf("invalid command: %s\n", line);
  }
  printf("> ");
}

