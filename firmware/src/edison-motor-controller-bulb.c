
#include <version.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "platform_config.h"
#include <utils/debug.h>
#include <contiki/core/sys/process.h>
#include <contiki/core/sys/etimer.h>
#include "edison-motor-controller-bulb.h"

#define SPI_STATE_RX_COMMAND       0x01
#define SPI_STATE_RX_DATA          0x02
#define SPI_STATE_TX_DATA          0x03
#define SPI_STATE_ERROR            0x04
#define SPI_STATE_COMPLETE         0x05

static const int8_t ENCODER_STATES[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
#define MOTOR_LEFT  0
#define MOTOR_RIGHT 1

PROCESS(watchdog_reset, "Watchdog Reset");
PROCESS(compass_update, "Compass Update");

volatile uint8_t lastCommand;
volatile uint8_t spiState;
volatile uint8_t lastMotorState[2];
volatile GPIO_PinState lastSpiCsState;

EdisonSocketConfig edisonSocketConfig = {
  .version = EDISON_SOCKET_VERSION,
  .driverUrl = "https://raw.githubusercontent.com/v5analytics/vertexium/" GIT_HASH "/benchmark/pom.xml"
};

volatile EdisonMotorCommandStatusResponse status = {
  .heading = EDISON_MOTOR_TARGET_HEADING_NOT_SET,
  .targetHeading = EDISON_MOTOR_TARGET_HEADING_NOT_SET,
  .speedLeft = 0,
  .distanceLeft = EDISON_MOTOR_DISTANCE_NOT_SET,
  .speedRight = 0,
  .distanceRight = EDISON_MOTOR_DISTANCE_NOT_SET
};

EdisonMotorCommandDrive driveCommand;

void spi_setup();
void spi_process();
void motor_processPinChange(uint8_t motor, GPIO_PinState chA, GPIO_PinState chB);
void motor_stop();
void motor_processDriveCommand();
uint32_t speedToCompareValue(uint16_t speed);
void spi_clear();

void setup() {
  printf("setup\n");

  lastCommand = EDISON_SOCKET_CMD_NOT_SET;
  spiState = SPI_STATE_COMPLETE;
  lastMotorState[MOTOR_LEFT] = 0;
  lastMotorState[MOTOR_RIGHT] = 0;
  lastSpiCsState = GPIO_PIN_SET;

  //HAL_IWDG_Start(&hiwdg);

  process_init();
  process_start(&etimer_process, NULL);
  //process_start(&watchdog_reset, NULL);
  process_start(&compass_update, NULL);

  debug_setup();
  printf("setup complete\n");
  printf("> ");
}

void loop() {
  process_run();
}

void spi_clear() {
  uint8_t temp[10];
  while (HAL_SPI_Receive(&SPI, temp, 10, 0) == HAL_OK);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
  if (pin == PIN_SPI1_CS_PIN) {
    GPIO_PinState pinState = HAL_GPIO_ReadPin(PIN_SPI1_CS_PORT, PIN_SPI1_CS_PIN);
    if (pinState == GPIO_PIN_RESET && lastSpiCsState == GPIO_PIN_SET) {
      spi_clear();
      spiState = SPI_STATE_RX_COMMAND;
      lastCommand = 0x00;
      HAL_SPI_Receive_DMA(&SPI, (uint8_t*)&lastCommand, 1);
    } else if (pinState == GPIO_PIN_SET) {
      spiState = SPI_STATE_COMPLETE;
      HAL_SPI_DMAStop(&SPI);
      spi_clear();
    }
    lastSpiCsState = pinState;
  } else if (pin == PIN_MOTORLCHA_PIN || pin == PIN_MOTORLCHB_PIN) {
    GPIO_PinState pinStateA = HAL_GPIO_ReadPin(PIN_MOTORLCHA_PORT, PIN_MOTORLCHA_PIN);
    GPIO_PinState pinStateB = HAL_GPIO_ReadPin(PIN_MOTORLCHB_PORT, PIN_MOTORLCHB_PIN);
    motor_processPinChange(MOTOR_LEFT, pinStateA, pinStateB);
  } else if (pin == PIN_MOTORRCHA_PIN || pin == PIN_MOTORRCHB_PIN) {
    GPIO_PinState pinStateA = HAL_GPIO_ReadPin(PIN_MOTORRCHA_PORT, PIN_MOTORRCHA_PIN);
    GPIO_PinState pinStateB = HAL_GPIO_ReadPin(PIN_MOTORRCHB_PORT, PIN_MOTORRCHB_PIN);
    motor_processPinChange(MOTOR_RIGHT, pinStateA, pinStateB);
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) {
  spi_process();
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi) {
  spi_process();
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
  spi_process();
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
  spiState = SPI_STATE_ERROR;
  printf("HAL_SPI_ErrorCallback 0x%08lx\n", HAL_SPI_GetError(hspi));
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  printf("HAL_UART_ErrorCallback 0x%08lx\n", HAL_UART_GetError(huart));
}

void spi_process() {
  if (spiState == SPI_STATE_RX_COMMAND) {
    if (lastCommand == EDISON_SOCKET_CMD_READ_CONFIG) {
      spiState = SPI_STATE_TX_DATA;
      HAL_SPI_Transmit_DMA(&SPI, (uint8_t*)&edisonSocketConfig, sizeof(edisonSocketConfig));
    } else if (lastCommand == EDISON_MOTOR_CMD_STATUS) {
      spiState = SPI_STATE_TX_DATA;
      HAL_SPI_Transmit_DMA(&SPI, (uint8_t*)&status, sizeof(status));
    } else if (lastCommand == EDISON_MOTOR_CMD_DRIVE) {
      spiState = SPI_STATE_RX_DATA;
      HAL_SPI_Receive_DMA(&SPI, (uint8_t*)&driveCommand, sizeof(driveCommand));
    } else {
      spiState = SPI_STATE_ERROR;
      printf("SPI: unknown rx command 0x%02x\n", lastCommand);
    }
  } else if (spiState == SPI_STATE_RX_DATA) {
    if (lastCommand == EDISON_MOTOR_CMD_DRIVE) {
      spiState = SPI_STATE_COMPLETE;
      motor_processDriveCommand();
    } else {
      spiState = SPI_STATE_ERROR;
      printf("SPI: unknown state RX command data 0x%02x\n", lastCommand);
    }
  } else if (spiState == SPI_STATE_TX_DATA) {
    if (lastCommand == EDISON_SOCKET_CMD_READ_CONFIG) {
      spiState = SPI_STATE_COMPLETE;
    } else if (lastCommand == EDISON_MOTOR_CMD_STATUS) {
      spiState = SPI_STATE_COMPLETE;
    } else {
      spiState = SPI_STATE_ERROR;
      printf("SPI: invalid tx command 0x%02x\n", lastCommand);
    }
  } else if (spiState == SPI_STATE_COMPLETE) {
  } else if (spiState == SPI_STATE_ERROR) {
  } else {
    printf("SPI: invalid tx state 0x%02x\n", spiState);
    spiState = SPI_STATE_ERROR;
  }
}

void motor_processPinChange(uint8_t motor, GPIO_PinState chA, GPIO_PinState chB) {
  volatile uint16_t* statusDistance = (motor == MOTOR_LEFT) ? &(status.distanceLeft) : &(status.distanceRight);
  uint8_t newState = (chA == GPIO_PIN_SET ? 0b10 : 0b00) | (chB == GPIO_PIN_SET ? 0b01 : 0b00);

  // nothing changed
  if ((lastMotorState[motor] & 0b0011) == newState) {
    return;
  }

  newState = ((lastMotorState[motor] << 2) | newState) & 0b1111;
  if (*statusDistance != EDISON_MOTOR_DISTANCE_NOT_SET && *statusDistance-- > 0) {
    int8_t move = ENCODER_STATES[newState];
    if (move != 0) {
      *statusDistance--;
      if (*statusDistance == 0) {
        motor_stop();
      }
    }
  }
  lastMotorState[motor] = newState;
}

void motor_stop() {
  HAL_GPIO_WritePin(PIN_MOTOREN_PORT, PIN_MOTOREN_PIN, GPIO_PIN_RESET);
  HAL_TIM_PWM_Stop(MOTOR_LEFT_PWM_HANDLE, MOTOR_LEFT_PWM_CHANNEL);
  HAL_TIM_PWM_Stop(MOTOR_RIGHT_PWM_HANDLE, MOTOR_RIGHT_PWM_CHANNEL);
  printf("motor_stop\n");
}

void motor_processDriveCommand() {
  status.speedLeft = driveCommand.speedLeft;
  status.distanceLeft = driveCommand.distanceLeft;
  status.speedRight = driveCommand.speedRight;
  status.distanceRight = driveCommand.distanceRight;
  status.targetHeading = driveCommand.targetHeading;

  HAL_GPIO_WritePin(PIN_MOTORLDIR_PORT, PIN_MOTORLDIR_PIN, driveCommand.speedLeft > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PIN_MOTORRDIR_PORT, PIN_MOTORRDIR_PIN, driveCommand.speedRight > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

  __HAL_TIM_SET_COMPARE(MOTOR_LEFT_PWM_HANDLE, MOTOR_LEFT_PWM_CHANNEL, speedToCompareValue(driveCommand.speedLeft));
  __HAL_TIM_SET_COMPARE(MOTOR_RIGHT_PWM_HANDLE, MOTOR_RIGHT_PWM_CHANNEL, speedToCompareValue(driveCommand.speedRight));

  HAL_TIM_PWM_Start(MOTOR_LEFT_PWM_HANDLE, MOTOR_LEFT_PWM_CHANNEL);
  HAL_TIM_PWM_Start(MOTOR_RIGHT_PWM_HANDLE, MOTOR_RIGHT_PWM_CHANNEL);

  HAL_GPIO_WritePin(PIN_MOTOREN_PORT, PIN_MOTOREN_PIN, GPIO_PIN_SET);

  printf("speedLeft: %d\n", driveCommand.speedLeft);
  printf("distanceLeft: %d\n", driveCommand.distanceLeft);
  printf("speedRight: %d\n", driveCommand.speedRight);
  printf("distanceRight: %d\n", driveCommand.distanceRight);
  printf("targetHeading: %d\n", driveCommand.targetHeading);
}

uint32_t speedToCompareValue(uint16_t speed) {
  return speed * 2;
}

void debug_processLine(const char* line) {
  if (strlen(line) == 0) {
  } else if (strcmp(line, "testiwdg") == 0) {
    printf("testing IWDG\n");
    while (1);
  } else if (strcmp(line, "status") == 0) {
    printf("heading: %d\n", status.heading);
    printf("targetHeading: %d\n", status.targetHeading);
    printf("speedLeft: %d\n", status.speedLeft);
    printf("distanceLeft: %d\n", status.distanceLeft);
    printf("speedRight: %d\n", status.speedRight);
    printf("distanceRight: %d\n", status.distanceRight);
  } else {
    printf("invalid debug command: %s\n", line);
  }
  printf("> ");
}

PROCESS_THREAD(watchdog_reset, ev, data) {
  static struct etimer et;

  PROCESS_BEGIN();

  while (1) {
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    //HAL_IWDG_Refresh(&hiwdg);
  }

  PROCESS_END();
}

PROCESS_THREAD(compass_update, ev, data) {
  static struct etimer et;

  PROCESS_BEGIN();

  while (1) {
    etimer_set(&et, COMPASS_TIMER_MS);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    // TODO update compass
    // status.heading = 0;
  }

  PROCESS_END();
}


