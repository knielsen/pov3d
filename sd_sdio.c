#include "ledtorus.h"

#include "stm324xg_eval_sdio_sd.h"

#include "ev_fat.h"


void
SD_LowLevel_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_AHB1PeriphClockCmd(SD_DETECT_GPIO_CLK, ENABLE);

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_SDIO);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SD_DETECT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  /*
    The SD card detect switch is open when no card is present, and connects to
    ground when card is inserted. So we need a pull-up on the pin.
  */
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SD_DETECT_GPIO_PORT, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, ENABLE);
  RCC_AHB1PeriphClockCmd(SD_SDIO_DMA_CLK, ENABLE);
}


void
SD_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  SDIO_ClockCmd(DISABLE);
  SDIO_SetPowerState(SDIO_PowerState_OFF);
  SDIO_DeInit();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, DISABLE);

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_MCO);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}


void
SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize)
{
  DMA_InitTypeDef SDDMA_InitStructure;

  DMA_ClearFlag(SD_SDIO_DMA_STREAM, SD_SDIO_DMA_FLAG_FEIF |
                SD_SDIO_DMA_FLAG_DMEIF | SD_SDIO_DMA_FLAG_TEIF |
                SD_SDIO_DMA_FLAG_HTIF | SD_SDIO_DMA_FLAG_TCIF);

  DMA_Cmd(SD_SDIO_DMA_STREAM, DISABLE);
  DMA_DeInit(SD_SDIO_DMA_STREAM);

  SDDMA_InitStructure.DMA_Channel = SD_SDIO_DMA_CHANNEL;
  SDDMA_InitStructure.DMA_PeripheralBaseAddr = SDIO_FIFO_ADDRESS;
  SDDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferSRC;
  SDDMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  SDDMA_InitStructure.DMA_BufferSize = BufferSize;
  SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  SDDMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
  DMA_Init(SD_SDIO_DMA_STREAM, &SDDMA_InitStructure);
  DMA_ITConfig(SD_SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_FlowControllerConfig(SD_SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

  DMA_Cmd(SD_SDIO_DMA_STREAM, ENABLE);
}

void
SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize)
{
  DMA_InitTypeDef SDDMA_InitStructure;

  DMA_ClearFlag(SD_SDIO_DMA_STREAM, SD_SDIO_DMA_FLAG_FEIF |
                SD_SDIO_DMA_FLAG_DMEIF | SD_SDIO_DMA_FLAG_TEIF |
                SD_SDIO_DMA_FLAG_HTIF | SD_SDIO_DMA_FLAG_TCIF);

  DMA_Cmd(SD_SDIO_DMA_STREAM, DISABLE);
  DMA_DeInit(SD_SDIO_DMA_STREAM);

  SDDMA_InitStructure.DMA_Channel = SD_SDIO_DMA_CHANNEL;
  SDDMA_InitStructure.DMA_PeripheralBaseAddr = SDIO_FIFO_ADDRESS;
  SDDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferDST;
  SDDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  SDDMA_InitStructure.DMA_BufferSize = BufferSize;
  SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  SDDMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
  DMA_Init(SD_SDIO_DMA_STREAM, &SDDMA_InitStructure);
  DMA_ITConfig(SD_SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_FlowControllerConfig(SD_SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

  DMA_Cmd(SD_SDIO_DMA_STREAM, ENABLE);
}


static const char *
sdio_error_name(SD_Error err)
{
  const char *err_name;
  switch(err)
  {
  case SD_CMD_CRC_FAIL: err_name = "SD_CMD_CRC_FAIL"; break;
  case SD_DATA_CRC_FAIL: err_name = "SD_DATA_CRC_FAIL"; break;
  case SD_CMD_RSP_TIMEOUT: err_name = "SD_CMD_RSP_TIMEOUT"; break;
  case SD_DATA_TIMEOUT: err_name = "SD_DATA_TIMEOUT"; break;
  case SD_TX_UNDERRUN: err_name = "SD_TX_UNDERRUN"; break;
  case SD_RX_OVERRUN: err_name = "SD_RX_OVERRUN"; break;
  case SD_START_BIT_ERR: err_name = "SD_START_BIT_ERR"; break;
  case SD_CMD_OUT_OF_RANGE: err_name = "SD_CMD_OUT_OF_RANGE"; break;
  case SD_ADDR_MISALIGNED: err_name = "SD_ADDR_MISALIGNED"; break;
  case SD_BLOCK_LEN_ERR: err_name = "SD_BLOCK_LEN_ERR"; break;
  case SD_ERASE_SEQ_ERR: err_name = "SD_ERASE_SEQ_ERR"; break;
  case SD_BAD_ERASE_PARAM: err_name = "SD_BAD_ERASE_PARAM"; break;
  case SD_WRITE_PROT_VIOLATION: err_name = "SD_WRITE_PROT_VIOLATION"; break;
  case SD_LOCK_UNLOCK_FAILED: err_name = "SD_LOCK_UNLOCK_FAILED"; break;
  case SD_COM_CRC_FAILED: err_name = "SD_COM_CRC_FAILED"; break;
  case SD_ILLEGAL_CMD: err_name = "SD_ILLEGAL_CMD"; break;
  case SD_CARD_ECC_FAILED: err_name = "SD_CARD_ECC_FAILED"; break;
  case SD_CC_ERROR: err_name = "SD_CC_ERROR"; break;
  case SD_GENERAL_UNKNOWN_ERROR: err_name = "SD_GENERAL_UNKNOWN_ERROR"; break;
  case SD_STREAM_READ_UNDERRUN: err_name = "SD_STREAM_READ_UNDERRUN"; break;
  case SD_STREAM_WRITE_OVERRUN: err_name = "SD_STREAM_WRITE_OVERRUN"; break;
  case SD_CID_CSD_OVERWRITE: err_name = "SD_CID_CSD_OVERWRITE"; break;
  case SD_WP_ERASE_SKIP: err_name = "SD_WP_ERASE_SKIP"; break;
  case SD_CARD_ECC_DISABLED: err_name = "SD_CARD_ECC_DISABLED"; break;
  case SD_ERASE_RESET: err_name = "SD_ERASE_RESET"; break;
  case SD_AKE_SEQ_ERROR: err_name = "SD_AKE_SEQ_ERROR"; break;
  case SD_INVALID_VOLTRANGE: err_name = "SD_INVALID_VOLTRANGE"; break;
  case SD_ADDR_OUT_OF_RANGE: err_name = "SD_ADDR_OUT_OF_RANGE"; break;
  case SD_SWITCH_ERROR: err_name = "SD_SWITCH_ERROR"; break;
  case SD_SDIO_DISABLED: err_name = "SD_SDIO_DISABLED"; break;
  case SD_SDIO_FUNCTION_BUSY: err_name = "SD_SDIO_FUNCTION_BUSY"; break;
  case SD_SDIO_FUNCTION_FAILED: err_name = "SD_SDIO_FUNCTION_FAILED"; break;
  case SD_SDIO_UNKNOWN_FUNCTION: err_name = "SD_SDIO_UNKNOWN_FUNCTION"; break;
  case SD_INTERNAL_ERROR: err_name = "SD_INTERNAL_ERROR"; break;
  case SD_NOT_CONFIGURED: err_name = "SD_NOT_CONFIGURED"; break;
  case SD_REQUEST_PENDING: err_name = "SD_REQUEST_PENDING"; break;
  case SD_REQUEST_NOT_APPLICABLE: err_name = "SD_REQUEST_NOT_APPLICABLE"; break;
  case SD_INVALID_PARAMETER: err_name = "SD_INVALID_PARAMETER"; break;
  case SD_UNSUPPORTED_FEATURE: err_name = "SD_UNSUPPORTED_FEATURE"; break;
  case SD_UNSUPPORTED_HW: err_name = "SD_UNSUPPORTED_HW"; break;
  case SD_ERROR: err_name = "SD_ERROR"; break;
  case SD_OK: err_name = "SD_OK"; break;
  default:
    err_name = "UNKNOWN";
  }
  return err_name;
}


void
SDIO_IRQHandler(void)
{
  SD_ProcessIRQSrc();
}

void
SD_SDIO_DMA_IRQHANDLER(void)
{
  /*
    There is a bug in the ST code library for SD/SDIO. It tests the wrong
    flag if DMA channel 6 is used (DMA2->LISR instead of DMA2->HISR).
    We work around it by inlining the (corrected) code here, rather than
    calling into SD_ProcessDMAIRQ().
  */
  if(SD_SDIO_DMA_ISR & (SD_SDIO_DMA_FLAG_TCIF & 0x0F7D0F7D))
  {
    extern volatile uint32_t DMAEndOfTransfer;
    DMAEndOfTransfer = 0x01;
    DMA_ClearFlag(SD_SDIO_DMA_STREAM, SD_SDIO_DMA_FLAG_TCIF|SD_SDIO_DMA_FLAG_FEIF);
  }
}


static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = SD_SDIO_DMA_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
  NVIC_Init(&NVIC_InitStructure);
}


void
setup_sd_sdio(void)
{
  uint32_t count;
  SD_Error err;
  static uint8_t buf[512*4];

  delay(2000000);
  NVIC_Configuration();
  delay(2000000);

  if (SD_Init() == SD_OK)
    serial_puts("SD ok!?!\r\n");
  else
    serial_puts("SD not ok :-/\r\n");
  delay(2000000);

  count = 0;
  while (count < 3)
  {
    println_uint32(count++);
    led_on();
    if ((err = SD_ReadMultiBlocks(buf, 0, 512, 4)) == SD_OK)
      serial_puts("SD multiread ok!?!\r\n");
    else
    {
      serial_puts("SD multiread not ok: ");
      serial_puts(sdio_error_name(err));
      serial_puts(" :-/\r\n");
    }
    if ((err = SD_WaitReadOperation()) == SD_OK)
      serial_puts("SD wait multi ok!?!\r\n");
    else
    {
      serial_puts("SD wait multi not ok: ");
      serial_puts(sdio_error_name(err));
      serial_puts(" :-/\r\n");
    }
    while (SD_GetStatus() != SD_TRANSFER_OK)
      ;
    led_off();

    serial_dump_buf(buf+512-16, 16);
    delay(10000000);
  }
}
