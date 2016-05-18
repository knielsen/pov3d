#include "ledtorus.h"

#ifdef ToDo_SDIO
#include "stm324xg_eval_sdio_sd.h"
#else
typedef uint32_t SD_Error;
#define SD_OK 0
#define SD_ERROR 1
#define SD_TRANSFER_OK 0
#define SD_TRANSFER_ERROR 1
#endif

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
#ifdef ToDo_SDIO
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
#else
  err_name = "SD_ERROR";
#endif
  return err_name;
}

void
SDIO_IRQHandler(void)
{
#ifdef ToDo_SDIO
  SD_ProcessIRQSrc();
#endif
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
#ifdef ToDo_SDIO
    extern volatile uint32_t DMAEndOfTransfer;
    DMAEndOfTransfer = 0x01;
#endif
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


static uint32_t sd_buf[512/sizeof(uint32_t)];
static struct ev_file_status sd_status;
static uint32_t file_sofar = 0;
static uint32_t file_length = 0;
static uint32_t range_start_sec = 0;
static uint32_t range_end_sec = 0;
static uint32_t cached_sector = 0;
static uint8_t have_cached_sector = 0;


static int
handle_stream_bytes(void)
{
  uint32_t sec = sd_status.st_stream_bytes.sec;
  uint32_t i, offset, len;
  SD_Error err;

  if (!have_cached_sector || cached_sector != sec)
  {
    /* Need to read the requested sector. */
    led_on();
    if ((err =
#ifdef ToDo_SDIO
         SD_ReadBlock((uint8_t*)sd_buf, (uint64_t)sec*512, 512)
#else
        SD_ERROR
#endif
         ) != SD_OK)
    {
      led_off();
      serial_puts("SD read error: ");
      serial_puts(sdio_error_name(err));
      serial_puts("\r\n");
      return 1;
    }
    if ((err =
#ifdef ToDo_SDIO
         SD_WaitReadOperation()
#else
         SD_ERROR
#endif
         ) != SD_OK)
    {
      led_off();
      serial_puts("SD wait single not ok: ");
      serial_puts(sdio_error_name(err));
      serial_puts("\r\n");
      return 1;
    }
    if (
#ifdef ToDo_SDIO
        SD_GetStatus()
#else
        SD_TRANSFER_ERROR
#endif
        != SD_TRANSFER_OK)
    {
      led_off();
      serial_puts("SD_GetStatus() not ok\r\n");
      return 1;
    }
    led_off();
    cached_sector = sec;
    have_cached_sector = 1;
  }

  offset = sd_status.st_stream_bytes.offset;
  len = sd_status.st_stream_bytes.len;
  for (i = 0; i < len; ++i)
    ev_file_stream_bytes(((uint8_t *)sd_buf)[offset + i], &sd_status);
  return 0;
}


int
open_file(const char *name)
{
  SD_Error err;
  int res;

  have_cached_sector = 0;                    /* In case of switch out card. */
#ifdef ToDo_SDIO
  if (SD_Detect() != SD_PRESENT)
#endif
  {
    serial_puts("No SD card present\r\n");
    return 1;
  }
  if ((err =
#ifdef ToDo_SDIO
       SD_Init()
#else
       SD_ERROR
#endif
       ) != SD_OK)
  {
    serial_puts("SD_Init() failed: ");
    serial_puts(sdio_error_name(err));
    serial_puts("\r\n");
    return 1;
  }

  sd_status.state = 0;
  for (;;)
  {
    res = ev_file_get_first_block(name, &sd_status);
    if (res < 0)
    {
      serial_puts("Error opening file: ");
      println_int32(res);
      return 1;
    }
    if (res == EV_FILE_ST_DONE)
      break;
    if (res == EV_FILE_ST_STREAM_BYTES)
    {
      if (handle_stream_bytes())
        return 1;
    }
  }
  file_sofar = 0;
  file_length = (sd_status.st_get_block_done.length+511)/512;
  range_end_sec = range_start_sec = sd_status.st_get_block_done.sector;

  return 0;
}


static int
get_next_block(void)
{
  int res;

  for (;;)
  {
    res = ev_file_get_next_block(&sd_status);
    if (res < 0)
    {
      serial_puts("Error opening file: ");
      println_int32(res);
      return 1;
    }
    if (res == EV_FILE_ST_DONE)
      break;
    if (res == EV_FILE_ST_STREAM_BYTES)
    {
      if (handle_stream_bytes())
        return 1;
    }
  }
  return 0;
}


static int
read_range(uint32_t **dest)
{
  uint32_t start = range_start_sec;
  uint32_t count = range_end_sec - start + 1;
  SD_Error err;

  led_on();
#ifdef ToDo_SDIO
  if (count == 1)
    err = SD_ReadBlock((uint8_t *)*dest, (uint64_t)start*512, 512);
  else
    err = SD_ReadMultiBlocks((uint8_t *)*dest, (uint64_t)start*512, 512, count);
#else
  err = SD_ERROR;
#endif
  if (err != SD_OK)
  {
    led_off();
    serial_puts("SD read error: ");
    serial_puts(sdio_error_name(err));
    serial_puts("\r\n");
    return 1;
  }
#ifdef ToDo_SDIO
  if ((err = SD_WaitReadOperation()) != SD_OK)
#endif
  {
    led_off();
    serial_puts("SD wait not ok: ");
    serial_puts(sdio_error_name(err));
    serial_puts("\r\n");
    return 1;
  }
#ifdef ToDo_SDIO
  if (SD_GetStatus() != SD_TRANSFER_OK)
#endif
  {
    led_off();
    serial_puts("SD_GetStatus() not ok\r\n");
    return 1;
  }
  led_off();
  *dest += count*(512/sizeof(uint32_t));
  return 0;
}


/*
  Read the next bunch of sectors into a buffer.

  Returns 0 on ok, -1 on end-of-file, 1 on error.
*/
int
read_sectors(uint32_t *buf, uint32_t count)
{
  uint32_t start = file_sofar;
  uint32_t *dest = buf;

  if (start >= file_length)
    return -1;
  /*
    A special case for the very first sector; we already found that in
    file_open(). This is detected by file_sofar == 0.
  */
  if (start != 0)
  {
    if (get_next_block())
      return 1;
    range_end_sec = range_start_sec = sd_status.st_get_block_done.sector;
  }
  ++file_sofar;

  for (;;)
  {
    uint32_t new_sec;

    if (file_sofar >= start + count)
      break;
    if (file_sofar >= file_length)
    {
      if (read_range(&dest))
        return 1;
      return -1;
    }
    if (get_next_block())
      return 1;
    new_sec = sd_status.st_get_block_done.sector;
    if (new_sec != range_end_sec + 1)
    {
      if (read_range(&dest))
        return 1;
      range_start_sec = new_sec;
    }
    range_end_sec = new_sec;
    ++file_sofar;
  }
  return read_range(&dest);
}


void
setup_sd_sdio(void)
{
  NVIC_Configuration();
}
