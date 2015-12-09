#include <pinout.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stm32l0xx_hal_flash.h>
#include <stm32l0xx_hal_spi.h>
#include <core_cmFunc.h>

#define APP_ADDRESS  0x08002000
#define SPI          (&hspi1)
#define SIGNATURE    0x12345678
#define PAGE_SIZE    ((uint32_t)0x00000080)
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
uint8_t buffer[PAGE_SIZE];
uint8_t spiState;
uint8_t* txData, *rxData;
uint32_t pageAddr;

void jumpToUserCode();
void assertIrq();
void deassertIrq();
void spi_process();
void spi_beginReceiveStartPage();
void spi_beginReceivePage();
void spi_flashPage();

void setup() {
  deassertIrq();

  while (HAL_GPIO_ReadPin(PIN_SPICS_PORT, PIN_SPICS_PIN) != GPIO_PIN_RESET);

  spiState = SPI_STATE_RECV_HEADER;
  txData = rxData = (uint8_t*)&flashHeader;
  memset(txData, 0, sizeof(FlashHeader));
  for(int i=0; i<sizeof(FlashHeader); i++) {
    txData[i] = i;
  }
  if (HAL_SPI_TransmitReceive_DMA(SPI, txData, rxData, sizeof(FlashHeader)) != HAL_OK) {
    jumpToUserCode();
    return;
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
  jumpToUserCode();
}

void spi_process() {
  switch(spiState) {
    case SPI_STATE_RECV_HEADER:
      if(flashHeader.signature != SIGNATURE) {
        jumpToUserCode();
        return;
      }
      txData = rxData = (uint8_t*)buffer;
      HAL_FLASH_Unlock();
      for(pageAddr = 0; pageAddr < flashHeader.size; pageAddr += PAGE_SIZE) {
	FLASH_Erase_Page(APP_ADDRESS + pageAddr);
      }
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

void spi_beginReceiveStartPage() {
  spiState = SPI_STATE_RECEIVE_START_PAGE;
  *((uint32_t*)txData) = pageAddr;
  assertIrq();
  if (HAL_SPI_TransmitReceive_DMA(SPI, txData, rxData, 4) != HAL_OK) {
    jumpToUserCode();
    return;
  }
}

void spi_beginReceivePage() {
  spiState = SPI_STATE_RECEIVE_PAGE;
  memset(txData, 0, PAGE_SIZE);
  for(int i=0; i<PAGE_SIZE; i++) {
    txData[i] = i;
  }
  deassertIrq();
  if (HAL_SPI_TransmitReceive_DMA(SPI, txData, rxData, PAGE_SIZE) != HAL_OK) {
    jumpToUserCode();
    return;
  }
}

void spi_flashPage() {
  for (uint32_t pageOffset = 0; pageOffset < PAGE_SIZE; pageOffset += WORD_SIZE) {
    if (HAL_FLASH_Program(TYPEPROGRAM_WORD, APP_ADDRESS + pageAddr + pageOffset, *((uint32_t*)(rxData + pageOffset))) != HAL_OK) {
      jumpToUserCode();
      return;
    }
  }
  pageAddr += PAGE_SIZE;
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
