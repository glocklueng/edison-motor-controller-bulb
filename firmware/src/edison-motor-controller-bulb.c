
#include <version.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "platform_config.h"
#include <utils/debug.h>
#include <utils/utils.h>
#include <utils/math.h>
#include <contiki/core/sys/process.h>
#include <contiki/core/sys/etimer.h>
#include <lis3mdl/lis3mdl.h>
#include <core_cmFunc.h>
#include "edison-motor-controller-bulb.h"

#define SPI_STATE_RX_COMMAND       0x01
#define SPI_STATE_RX_DATA          0x02
#define SPI_STATE_TX_DATA          0x03
#define SPI_STATE_ERROR            0x04
#define SPI_STATE_COMPLETE         0x05

#define EEPROM_SETTINGS_ADDRESS    (DATA_EEPROM_BASE + 0x0000)
#define SETTINGS_SIGNATURE         0x6964795b

//#define DEBUG_OUT(format, ...) printf("%s:%d: " format, __FILE__, __LINE__, __VA_ARGS__)
#define DEBUG_OUT(format, ...)

static const int8_t ENCODER_STATES[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
#define MOTOR_LEFT  0
#define MOTOR_RIGHT 1

//PROCESS(watchdog_reset, "Watchdog Reset");
PROCESS(compass_update, "Compass Update");
PROCESS(motor_update, "Motor Update");

LIS3MDL compass;

typedef struct {
  uint32_t signature;
  int16_t compassMin[3];
  int16_t compassMax[3];
} EdisonMotorEEPROM;

volatile uint8_t lastCommand;
volatile uint8_t spiState;
volatile uint8_t lastMotorState[2];
volatile GPIO_PinState lastSpiCsState;
volatile EdisonMotorEEPROM eepromSettings;

int16_t debug_speedLeft;
int16_t debug_speedRight;
uint16_t debug_distanceLeft;
uint16_t debug_distanceRight;
uint16_t debug_rotation;

EdisonSocketConfig edisonSocketConfig = {
  .version = EDISON_SOCKET_VERSION,
  .driverUrl = "https://raw.githubusercontent.com/v5analytics/vertexium/" GIT_HASH "/benchmark/pom.xml"
};

volatile EdisonMotorCommandStatusResponse status = {
  .heading = EDISON_MOTOR_UNKNOWN_HEADING,
  .rotation = EDISON_MOTOR_ROTATION_NOT_SET,
  .speedLeft = 0,
  .distanceLeft = EDISON_MOTOR_DISTANCE_NOT_SET,
  .speedRight = 0,
  .distanceRight = EDISON_MOTOR_DISTANCE_NOT_SET
};

volatile EdisonMotorCommandDrive driveCommand;

void spi_process();
void motor_processPinChange(uint8_t motor, GPIO_PinState chA, GPIO_PinState chB);
void motor_stop();
void motor_processDriveCommand();
uint32_t speedToCompareValue(int16_t speed);
void spi_clear();
HAL_StatusTypeDef compass_readXYHeading(uint16_t* heading);
int16_t compass_scale(int32_t value, int32_t min, int32_t max);
int16_t reduceSpeed(int16_t speed, uint16_t percent);
void reduceDriveCommandSpeed(uint16_t percent);
void clearCalibration();
void loadSettings();
void saveSettings();

void setup() {
  DEBUG_OUT("setup\n");

  HAL_SPI_DMAStop(&SPI);

  lastCommand = EDISON_SOCKET_CMD_NOT_SET;
  spiState = SPI_STATE_COMPLETE;
  lastMotorState[MOTOR_LEFT] = 0;
  lastMotorState[MOTOR_RIGHT] = 0;
  lastSpiCsState = GPIO_PIN_SET;

  debug_speedLeft = 100;
  debug_speedRight = 100;
  debug_distanceLeft = 200;
  debug_distanceRight = 200;
  debug_rotation = EDISON_MOTOR_UNKNOWN_HEADING;

  //HAL_IWDG_Start(&hiwdg);
  LIS3MDL_setup(&compass, &hi2c1, LIS3MDL_ADDRESS1);
  LIS3MDL_reset(&compass);
  LIS3MDL_enableTemperature(&compass, false);
  LIS3MDL_setPerformance(&compass, LIS3MDL_PERFORMANCE_HIGH);
  LIS3MDL_setDateRate(&compass, LIS3MDL_DATA_RATE_80_HZ);
  LIS3MDL_setScale(&compass, LIS3MDL_SCALE_4_GAUSS);
  LIS3MDL_setMode(&compass, LIS3MDL_MODE_CONTINUOUS);

  // must come after compass initialized because it changes the compass state
  loadSettings();

  process_init();
  process_start(&etimer_process, NULL);
  //process_start(&watchdog_reset, NULL);
  process_start(&compass_update, NULL);
  process_start(&motor_update, NULL);

  debug_setup();
  DEBUG_OUT("setup complete\n");
  DEBUG_OUT("> ");

  __enable_irq();
}

void loop() {
  process_run();
}

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
  if (pin == PIN_SPICS_PIN) {
    GPIO_PinState pinState = HAL_GPIO_ReadPin(PIN_SPICS_PORT, PIN_SPICS_PIN);
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
  }
}

void spi_clear() {
  uint8_t temp[10];
  while (HAL_SPI_Receive(&SPI, temp, 10, 0) == HAL_OK);
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
  DEBUG_OUT("HAL_SPI_ErrorCallback 0x%02x\n", (uint8_t)HAL_SPI_GetError(hspi));
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  DEBUG_OUT("HAL_UART_ErrorCallback 0x%02x\n", (uint8_t)HAL_UART_GetError(huart));
}

void spi_process() {
  if (spiState == SPI_STATE_RX_COMMAND) {
    if (lastCommand == EDISON_SOCKET_CMD_READ_CONFIG) {
      spiState = SPI_STATE_TX_DATA;
      HAL_SPI_Transmit_DMA(&SPI, (uint8_t*)&edisonSocketConfig, sizeof(edisonSocketConfig));
    } else if (lastCommand == EDISON_MOTOR_CMD_STATUS) {
      spiState = SPI_STATE_TX_DATA;
      HAL_SPI_Transmit_DMA(&SPI, (uint8_t*)&status, sizeof(status));
    } else if (lastCommand == EDISON_MOTOR_CMD_CLEAR_SETTINGS) {
      spiState = SPI_STATE_COMPLETE;
      clearCalibration();
    } else if (lastCommand == EDISON_MOTOR_CMD_SAVE_SETTINGS) {
      spiState = SPI_STATE_COMPLETE;
      saveSettings();
    } else if (lastCommand == EDISON_MOTOR_CMD_DRIVE) {
      spiState = SPI_STATE_RX_DATA;
      HAL_SPI_Receive_DMA(&SPI, (uint8_t*)&driveCommand, sizeof(driveCommand));
    } else {
      spiState = SPI_STATE_ERROR;
      DEBUG_OUT("SPI: unknown rx command 0x%02x\n", lastCommand);
    }
  } else if (spiState == SPI_STATE_RX_DATA) {
    if (lastCommand == EDISON_MOTOR_CMD_DRIVE) {
      spiState = SPI_STATE_COMPLETE;
      motor_processDriveCommand();
    } else {
      spiState = SPI_STATE_ERROR;
      DEBUG_OUT("SPI: unknown state RX command data 0x%02x\n", lastCommand);
    }
  } else if (spiState == SPI_STATE_TX_DATA) {
    if (lastCommand == EDISON_SOCKET_CMD_READ_CONFIG) {
      spiState = SPI_STATE_COMPLETE;
    } else if (lastCommand == EDISON_MOTOR_CMD_STATUS) {
      spiState = SPI_STATE_COMPLETE;
    } else {
      spiState = SPI_STATE_ERROR;
      DEBUG_OUT("SPI: invalid tx command 0x%02x\n", lastCommand);
    }
  } else if (spiState == SPI_STATE_COMPLETE) {
  } else if (spiState == SPI_STATE_ERROR) {
  } else {
    DEBUG_OUT("SPI: invalid tx state 0x%02x\n", spiState);
    spiState = SPI_STATE_ERROR;
  }
}

PROCESS_THREAD(motor_update, ev, data) {
  GPIO_PinState pinStateA, pinStateB;

  PROCESS_BEGIN();

  while (1) {
    pinStateA = HAL_GPIO_ReadPin(PIN_MOTORLCHA_PORT, PIN_MOTORLCHA_PIN);
    pinStateB = HAL_GPIO_ReadPin(PIN_MOTORLCHB_PORT, PIN_MOTORLCHB_PIN);
    motor_processPinChange(MOTOR_LEFT, pinStateA, pinStateB);

    pinStateA = HAL_GPIO_ReadPin(PIN_MOTORRCHA_PORT, PIN_MOTORRCHA_PIN);
    pinStateB = HAL_GPIO_ReadPin(PIN_MOTORRCHB_PORT, PIN_MOTORRCHB_PIN);
    motor_processPinChange(MOTOR_RIGHT, pinStateA, pinStateB);

    PROCESS_PAUSE();
  }

  PROCESS_END();
}

void motor_processPinChange(uint8_t motor, GPIO_PinState chA, GPIO_PinState chB) {
  uint16_t statusDistance = (motor == MOTOR_LEFT) ? status.distanceLeft : status.distanceRight;
  uint8_t newState = (chA == GPIO_PIN_SET ? 0b10 : 0b00) | (chB == GPIO_PIN_SET ? 0b01 : 0b00);

  // nothing changed
  if ((lastMotorState[motor] & 0b0011) == newState) {
    return;
  }

  newState = ((lastMotorState[motor] << 2) | newState) & 0b1111;
  if (statusDistance != EDISON_MOTOR_DISTANCE_NOT_SET && statusDistance > 0) {
    int8_t move = ENCODER_STATES[newState];
    if (move != 0) {
      statusDistance--;
      if (motor == MOTOR_LEFT) {
        status.distanceLeft = statusDistance;
      } else {
        status.distanceRight = statusDistance;
      }
      if (statusDistance == 0) {
        motor_stop();
      }
    }
  }
  lastMotorState[motor] = newState;
}

void motor_stop() {
  HAL_GPIO_WritePin(PIN_MOTOREN_PORT, PIN_MOTOREN_PIN, GPIO_PIN_RESET);
  HAL_TIM_Base_Stop_IT(MOTOR_LEFT_PWM_HANDLE);
  HAL_TIM_OC_Stop_IT(MOTOR_LEFT_PWM_HANDLE, MOTOR_LEFT_PWM_CHANNEL);
  HAL_TIM_OC_Stop_IT(MOTOR_RIGHT_PWM_HANDLE, MOTOR_RIGHT_PWM_CHANNEL);
  status.speedLeft = driveCommand.speedLeft = 0;
  status.distanceLeft = driveCommand.distanceLeft = EDISON_MOTOR_DISTANCE_NOT_SET;
  status.speedRight = driveCommand.speedRight = 0;
  status.distanceRight = driveCommand.distanceRight = EDISON_MOTOR_DISTANCE_NOT_SET;
  status.rotation = driveCommand.rotation = EDISON_MOTOR_ROTATION_NOT_SET;
  DEBUG_OUT("motor_stop\n");
}

void motor_processDriveCommand() {
  // cancel current interrupts to prevent speed jumping
  HAL_GPIO_WritePin(PIN_MOTOREN_PORT, PIN_MOTOREN_PIN, GPIO_PIN_RESET);
  HAL_TIM_Base_Stop_IT(MOTOR_LEFT_PWM_HANDLE);
  HAL_TIM_OC_Stop_IT(MOTOR_LEFT_PWM_HANDLE, MOTOR_LEFT_PWM_CHANNEL);
  HAL_TIM_OC_Stop_IT(MOTOR_RIGHT_PWM_HANDLE, MOTOR_RIGHT_PWM_CHANNEL);

  if (abs(driveCommand.speedLeft) < MOTOR_MIN_SPEED && abs(driveCommand.speedRight) < MOTOR_MIN_SPEED) {
    motor_stop();
    return;
  }
  if (driveCommand.rotation == EDISON_MOTOR_ROTATION_NOT_SET && driveCommand.distanceLeft == EDISON_MOTOR_DISTANCE_NOT_SET && driveCommand.distanceRight == EDISON_MOTOR_DISTANCE_NOT_SET) {
    motor_stop();
    return;
  }

  status.speedLeft = driveCommand.speedLeft;
  status.distanceLeft = driveCommand.distanceLeft;
  status.speedRight = driveCommand.speedRight;
  status.distanceRight = driveCommand.distanceRight;
  status.rotation = driveCommand.rotation;

  HAL_GPIO_WritePin(PIN_MOTORLDIR_PORT, PIN_MOTORLDIR_PIN, driveCommand.speedLeft > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PIN_MOTORRDIR_PORT, PIN_MOTORRDIR_PIN, driveCommand.speedRight > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);

  __HAL_TIM_SetCounter(MOTOR_LEFT_PWM_HANDLE, 0);
  __HAL_TIM_SetCompare(MOTOR_LEFT_PWM_HANDLE, MOTOR_LEFT_PWM_CHANNEL, speedToCompareValue(driveCommand.speedLeft));
  __HAL_TIM_SetCounter(MOTOR_RIGHT_PWM_HANDLE, 0);
  __HAL_TIM_SetCompare(MOTOR_RIGHT_PWM_HANDLE, MOTOR_RIGHT_PWM_CHANNEL, speedToCompareValue(driveCommand.speedRight));

  HAL_TIM_Base_Start_IT(MOTOR_LEFT_PWM_HANDLE);
  HAL_TIM_OC_Start_IT(MOTOR_LEFT_PWM_HANDLE, MOTOR_LEFT_PWM_CHANNEL);
  HAL_TIM_OC_Start_IT(MOTOR_RIGHT_PWM_HANDLE, MOTOR_RIGHT_PWM_CHANNEL);

  HAL_GPIO_WritePin(PIN_MOTOREN_PORT, PIN_MOTOREN_PIN, GPIO_PIN_SET);

  DEBUG_OUT("speedLeft: %d\n", driveCommand.speedLeft);
  DEBUG_OUT("distanceLeft: %d\n", driveCommand.distanceLeft);
  DEBUG_OUT("speedRight: %d\n", driveCommand.speedRight);
  DEBUG_OUT("distanceRight: %d\n", driveCommand.distanceRight);
  DEBUG_OUT("rotation: %d\n", driveCommand.rotation);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  HAL_GPIO_WritePin(PIN_MOTORLPWM_PORT, PIN_MOTORLPWM_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PIN_MOTORRPWM_PORT, PIN_MOTORRPWM_PIN, GPIO_PIN_SET);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim->Channel == MOTOR_LEFT_PWM_IT_CHANNEL) {
    HAL_GPIO_WritePin(PIN_MOTORLPWM_PORT, PIN_MOTORLPWM_PIN, GPIO_PIN_RESET);
  } else if (htim->Channel == MOTOR_RIGHT_PWM_IT_CHANNEL) {
    HAL_GPIO_WritePin(PIN_MOTORRPWM_PORT, PIN_MOTORRPWM_PIN, GPIO_PIN_RESET);
  }
}

uint32_t speedToCompareValue(int16_t speed) {
  if (speed < 0) {
    speed = -speed;
  }
  return speed * 2;
}

void debug_processLine(const char* line) {
  if (strlen(line) == 0) {
  } else if (strcmp(line, "testiwdg") == 0) {
    DEBUG_OUT("testing IWDG\n");
    while (1);
  } else if (strcmp(line, "status") == 0) {
    DEBUG_OUT("heading: %d\n", status.heading);
    DEBUG_OUT("rotation: %d\n", status.rotation);
    DEBUG_OUT("speedLeft: %d\n", status.speedLeft);
    DEBUG_OUT("distanceLeft: %d\n", status.distanceLeft);
    DEBUG_OUT("speedRight: %d\n", status.speedRight);
    DEBUG_OUT("distanceRight: %d\n", status.distanceRight);
  } else if (strcmp(line, "s") == 0 || strcmp(line, "stop") == 0) {
    motor_stop();
  } else if (strncmp(line, "sl", 2) == 0) {
    debug_speedLeft = atoi(line + 2);
  } else if (strncmp(line, "sr", 2) == 0) {
    debug_speedRight = atoi(line + 2);
  } else if (strncmp(line, "dl", 2) == 0) {
    debug_distanceLeft = atoi(line + 2);
  } else if (strncmp(line, "dr", 2) == 0) {
    debug_distanceRight = atoi(line + 2);
  } else if (strcmp(line, "d") == 0 || strcmp(line, "drive") == 0) {
    driveCommand.speedLeft = debug_speedLeft;
    driveCommand.distanceLeft = debug_distanceLeft;
    driveCommand.speedRight = debug_speedRight;
    driveCommand.distanceRight = debug_distanceRight;
    driveCommand.rotation = debug_rotation;
    motor_processDriveCommand();
  } else {
    DEBUG_OUT("invalid debug command: %s\n", line);
  }
  DEBUG_OUT("> ");
}
/*
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
*/
PROCESS_THREAD(compass_update, ev, data) {
  static struct etimer et;
  HAL_StatusTypeDef s;
  uint16_t heading;
  uint8_t compassStatus;

  PROCESS_BEGIN();

  while (1) {
    etimer_set(&et, COMPASS_TIMER_MS);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    s = LIS3MDL_readStatus(&compass, &compassStatus);
    if (s != HAL_OK) {
      DEBUG_OUT("could not read compass status: %d\n", s);
      continue;
    }
    if (testBits(compassStatus, LIS3MDL_STATUS_ZYXDA)) {
      s = compass_readXYHeading(&heading);
      if (s != HAL_OK) {
        DEBUG_OUT("could not read XY heading: %d\n", s);
        continue;
      }
      if (status.rotation != EDISON_MOTOR_ROTATION_NOT_SET) {
        status.rotation += smallestDeltaBetweenAnglesInDegrees(heading, status.heading);
        if (status.speedLeft > status.speedRight) {
          if (status.rotation < 2) {
            motor_stop();
          }
        } else {
          if (status.rotation > -2) {
            motor_stop();
          }
        }
      }
      status.heading = heading;
      DEBUG_OUT("heading: %d\n", status.heading);
    }
  }

  PROCESS_END();
}

HAL_StatusTypeDef compass_readXYHeading(uint16_t* heading) {
  HAL_StatusTypeDef status;
  int16_t x, y;
  int16_t scaledX, scaledY;

  status = LIS3MDL_readAxis(&compass, LIS3MDL_AXIS_X, &x);
  if (status != HAL_OK) {
    return status;
  }

  status = LIS3MDL_readAxis(&compass, LIS3MDL_AXIS_Y, &y);
  if (status != HAL_OK) {
    return status;
  }

  scaledX = compass_scale(x, compass.min[LIS3MDL_AXIS_X], compass.max[LIS3MDL_AXIS_X]);
  scaledY = compass_scale(y, compass.min[LIS3MDL_AXIS_Y], compass.max[LIS3MDL_AXIS_Y]);

  *heading = 360 - trig_int16_atan2deg(scaledY, scaledX);
  DEBUG_OUT("%d, %d, %d, %d, %d, %d, %d, %d, %d\n", x, scaledX, compass.min[LIS3MDL_AXIS_X], compass.max[LIS3MDL_AXIS_X], y, scaledY, compass.min[LIS3MDL_AXIS_Y], compass.max[LIS3MDL_AXIS_Y], *heading);

  return HAL_OK;
}

int16_t compass_scale(int32_t value, int32_t min, int32_t max) {
  value = value - ((max + min) / 2); // center around mid point
  value = value * (20000 / (max - min)); // scale from -10000 to 10000
  return value;
}

int16_t reduceSpeed(int16_t speed, uint16_t percent) {
  int32_t s = ((int32_t)speed * (100 - percent)) / 100;
  if (s < 0 && s > -MOTOR_MIN_SPEED) {
    s = -MOTOR_MIN_SPEED;
  } else if (s > 0 && s < MOTOR_MIN_SPEED) {
    s = MOTOR_MIN_SPEED;
  }
  return s;
}

void reduceDriveCommandSpeed(uint16_t percent) {
  driveCommand.speedLeft = reduceSpeed(driveCommand.speedLeft, percent);
  driveCommand.speedRight = reduceSpeed(driveCommand.speedRight, percent);
  motor_processDriveCommand();
}

void clearCalibration() {
  LIS3MDL_clearMinMax(&compass);
  for (int axis = 0; axis < 3; axis++) {
    eepromSettings.compassMin[axis] = 32767;
    eepromSettings.compassMax[axis] = -32768;
  }
}

void loadSettings() {
  memcpy(&eepromSettings, (uint32_t*)EEPROM_SETTINGS_ADDRESS, sizeof(eepromSettings));;
  if (eepromSettings.signature != SETTINGS_SIGNATURE) {
    return;
  }

  for (int axis = 0; axis < 3; axis++) {
    compass.min[axis] = eepromSettings.compassMin[axis];
    compass.max[axis] = eepromSettings.compassMax[axis];
  }
}

void saveSettings() {
  eepromSettings.signature = SETTINGS_SIGNATURE;
  for (int axis = 0; axis < 3; axis++) {
    eepromSettings.compassMin[axis] = compass.min[axis];
    eepromSettings.compassMax[axis] = compass.max[axis];
  }

  uint32_t addr = EEPROM_SETTINGS_ADDRESS;
  uint32_t* p = &eepromSettings;
  HAL_FLASHEx_DATAEEPROM_Unlock();
  HAL_FLASHEx_DATAEEPROM_Erase(addr);
  for (int i = 0; i < sizeof(eepromSettings); i += 4) {
    HAL_FLASHEx_DATAEEPROM_Program(TYPEPROGRAM_WORD, addr, *p);
    addr += 4;
    p++;
  }
  HAL_FLASHEx_DATAEEPROM_Lock();
}

