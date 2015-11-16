
#include "platform_config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <utils/debug.h>
#include <utils/time.h>
#include "edison-socket.h"

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
  
  if (strlen(line) == 0) {
  } else if(strcmp(line, "g") == 0) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    sleep_ms(10);
    cmd = 0x01;
    HAL_SPI_Transmit(&SPI, &cmd, 1, MAX_TIMEOUT);
    sleep_ms(10);
    HAL_SPI_Receive(&SPI, (uint8_t*)&config, sizeof(config), MAX_TIMEOUT);
    printf("version: 0x%02x\n", config.version);
    printf("url: %.*s\n", sizeof(config.driverUrl), config.driverUrl);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  } else {
    printf("invalid command: %s\n", line);
  }
  printf("> ");
}

