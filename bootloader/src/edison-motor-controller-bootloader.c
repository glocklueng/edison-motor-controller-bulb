#include <pinout.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stm32l0xx_hal_flash.h>
#include <stm32l0xx_hal_flash_ex.h>
#include <stm32l0xx_hal_spi.h>
#include <core_cmFunc.h>

#define APP_ADDRESS  (FLASH_BASE + 0x3000)
#define SPI          (&hspi1)
#define SIGNATURE    0x12345678
#define FLASH_PAGE_SIZE    ((uint32_t)0x00000080)
#define WORD_SIZE    4
#define TIMEOUT      10000

typedef struct {
  uint32_t signature;
  uint32_t crc32;
  uint32_t size;
} FlashHeader;

#define SPI_STATE_RECV_HEADER        1
#define SPI_STATE_RECEIVE_START_PAGE 2
#define SPI_STATE_RECEIVE_PAGE       3

extern SPI_HandleTypeDef hspi1;
FlashHeader flashHeader;
uint8_t buffer[FLASH_PAGE_SIZE];
uint8_t spiState;
uint32_t pageAddr;

void jumpToUserCode();
void assertIrq();
void deassertIrq();
void spi_process();
void spi_beginReceiveStartPage();
void spi_beginReceivePage();
void spi_flashPage();
void eraseProgramMemory();

void setup() {
  deassertIrq();

  while (HAL_GPIO_ReadPin(PIN_SPICS_PORT, PIN_SPICS_PIN) != GPIO_PIN_RESET);

  spiState = SPI_STATE_RECV_HEADER;
  uint8_t* rxData = (uint8_t*)&flashHeader;
  memset(rxData, 0, sizeof(FlashHeader));
  if (HAL_SPI_TransmitReceive_DMA(SPI, rxData, rxData, sizeof(FlashHeader)) != HAL_OK) {
    while (1);
  }
}

void loop() {
  while (1);
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
  while (1);
}

void spi_process() {
  switch (spiState) {
  case SPI_STATE_RECV_HEADER:
    if (flashHeader.signature != SIGNATURE) {
      while (1);
    }
    eraseProgramMemory();
    pageAddr = 0;
    spi_beginReceiveStartPage();
    break;

  case SPI_STATE_RECEIVE_START_PAGE:
    spi_beginReceivePage();
    break;

  case SPI_STATE_RECEIVE_PAGE:
    spi_flashPage();
    spi_beginReceiveStartPage();
    break;
  }
}

void eraseProgramMemory() {
  FLASH_EraseInitTypeDef eraseInitStruct;
  uint32_t pageError;

  HAL_FLASH_Unlock();

  eraseInitStruct.TypeErase = TYPEERASE_PAGEERASE;
  eraseInitStruct.Page = APP_ADDRESS;
  eraseInitStruct.NbPages = flashHeader.size / FLASH_PAGE_SIZE;
  if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK) {
    while (1);
  }
  if (FLASH_WaitForLastOperation(TIMEOUT) != HAL_OK) {
    while (1);
  }
}

void spi_beginReceiveStartPage() {
  spiState = SPI_STATE_RECEIVE_START_PAGE;
  *((uint32_t*)buffer) = pageAddr;
  if (HAL_SPI_TransmitReceive_DMA(SPI, buffer, buffer, 4) != HAL_OK) {
    while (1);
  }
  assertIrq(); // this needs to come after starting the DMA to ensure the DMA is ready to accept bytes
}

void spi_beginReceivePage() {
  spiState = SPI_STATE_RECEIVE_PAGE;
  memset(buffer, 0, FLASH_PAGE_SIZE);
  if (HAL_SPI_TransmitReceive_DMA(SPI, buffer, buffer, FLASH_PAGE_SIZE) != HAL_OK) {
    while (1);
  }
  deassertIrq(); // this needs to come after starting the DMA to ensure the DMA is ready to accept bytes
}

void spi_flashPage() {
  for (uint32_t pageOffset = 0; pageOffset < FLASH_PAGE_SIZE; pageOffset += WORD_SIZE) {
    uint32_t word = *((uint32_t*)(&buffer[pageOffset]));
    if (HAL_FLASH_Program(TYPEPROGRAM_WORD, APP_ADDRESS + pageAddr + pageOffset, word) != HAL_OK) {
      while (1);
    }
  }
  pageAddr += FLASH_PAGE_SIZE;
}

void assertIrq() {
  HAL_GPIO_WritePin(PIN_NIRQ_PORT, PIN_NIRQ_PIN, GPIO_PIN_RESET);
}

void deassertIrq() {
  HAL_GPIO_WritePin(PIN_NIRQ_PORT, PIN_NIRQ_PIN, GPIO_PIN_SET);
}

void jumpToUserCode() {
  typedef void (*funcPtr)(void);

  uint32_t jumpAddr = *(uint32_t*)(APP_ADDRESS + 0x04);  /* reset ptr in vector table */
  funcPtr usrMain = (funcPtr) jumpAddr;

  HAL_FLASH_Lock();
  __set_MSP(*(uint32_t*) APP_ADDRESS);

  usrMain();
}
