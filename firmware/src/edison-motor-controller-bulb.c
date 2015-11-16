
#include <version.h>
#include <stdio.h>
#include <string.h>
#include "platform_config.h"
#include <utils/debug.h>
#include <contiki/core/sys/process.h>
#include <contiki/core/sys/etimer.h>
#include "edison-socket.h"

void spi_setup();

PROCESS(watchdog_reset, "Watchdog Reset");
PROCESS(compass_update, "Compass Update");

uint32_t currentCompassHeading = 0;
uint8_t spiTxData[100];
uint8_t spiRxData[100];
EdisonSocketConfig edisonSocketConfig = {
  .version = EDISON_SOCKET_VERSION,
  .driverUrl = "https://raw.githubusercontent.com/v5analytics/vertexium/" GIT_HASH "/benchmark/pom.xml"
};

void setup() {
  printf("setup\n");

  //HAL_IWDG_Start(&hiwdg);

  process_init();
  process_start(&etimer_process, NULL);
  process_start(&watchdog_reset, NULL);
  process_start(&compass_update, NULL);

  debug_setup();
  printf("setup complete\n");
  printf("> ");
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
  if(pin == PIN_SPI1_CS_PIN) {
    spiTxData[0] = 0x00;
    HAL_SPI_Receive_IT(&SPI, spiRxData, 1);
  }
  //printf("HAL_GPIO_EXTI_Callback %d\n", pin);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  printf("HAL_SPI_TxCpltCallback\n");
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
  if(spiRxData[0] == EDISON_SOCKET_CMD_READ_CONFIG) {
    HAL_SPI_Transmit_IT(&SPI, (uint8_t*)&edisonSocketConfig, sizeof(edisonSocketConfig));
  } else {
    spiTxData[0] = 0x00;
    HAL_SPI_Transmit_IT(&SPI, spiTxData, 1);
  }
  //printf("HAL_SPI_RxCpltCallback 0x%02x\n", spiRxData[0]);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  printf("HAL_SPI_TxRxCpltCallback\n");
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  printf("HAL_SPI_ErrorCallback\n");
}

void loop() {
  process_run();
}

void debug_processLine(const char* line) {
  if (strlen(line) == 0) {
  } else if(strcmp(line, "testiwdg") == 0) {
    printf("testing IWDG\n");
    while(1);
  } else {
    printf("invalid debug command: %s\n", line);
  }
  printf("> ");
}

PROCESS_THREAD(watchdog_reset, ev, data) {
  static struct etimer et;

  PROCESS_BEGIN();
  
  while(1) {
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    //HAL_IWDG_Refresh(&hiwdg);
  }

  PROCESS_END();
}

PROCESS_THREAD(compass_update, ev, data) {
  static struct etimer et;

  PROCESS_BEGIN();
  
  while(1) {
    etimer_set(&et, 100);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    // TODO update compass
    currentCompassHeading = 0;
  }

  PROCESS_END();
}

