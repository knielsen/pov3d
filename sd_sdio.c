#include "ledtorus.h"

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/sdio.h>

#include "sdcard.h"
#include "ev_fat.h"


/* We use SDIO on DMA2 Stream 6. */
#define DMA_PERIPH DMA2
#define DMA_STREAM_INSTANCE DMA_STREAM6
#define DMA_ISR dma2_stream6_isr
#define DMA_IRQ NVIC_DMA2_STREAM6_IRQ
#define DMA_RCC_PERIPH RCC_DMA2

/* SD card detect pin on PA15. */
#define SD_DETECT_RCC_PERIPH RCC_GPIOA
#define SD_DETECT_GPIO GPIOA
#define SD_DETECT_PIN GPIO15


void
sdio_isr(void)
{
  HAL_SD_IRQHandler();
}


void
DMA_ISR(void)
{
  uint32_t direction;

  if (dma_get_interrupt_flag(DMA_PERIPH, DMA_STREAM_INSTANCE, DMA_TCIF))
  {
    direction = DMA_SCR(DMA_PERIPH, DMA_STREAM_INSTANCE) & DMA_SxCR_DIR_MASK;
    // ToDo: This is actually redundant, as the two callbacks do the same thing...
    if (direction == DMA_SxCR_DIR_PERIPHERAL_TO_MEM)
      SD_DMA_RxCplt();
    else
      SD_DMA_TxCplt();
    dma_clear_interrupt_flags(DMA_PERIPH, DMA_STREAM_INSTANCE, DMA_TCIF);
  }
}


static void
setup_sd_gpio_interrupts_dma(void)
{
  rcc_periph_clock_enable(SD_DETECT_RCC_PERIPH);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_GPIOD);
  rcc_periph_clock_enable(RCC_SDIO);
  rcc_periph_clock_enable(DMA_RCC_PERIPH);

  /*
    The SD card detect switch is open when no card is present, and connects to
    ground when card is inserted. So we need a pull-up on the pin.
  */
  gpio_mode_setup(SD_DETECT_GPIO, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, SD_DETECT_PIN);
  gpio_set_output_options(SD_DETECT_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, SD_DETECT_PIN);

  /* SDIO D0-D3 on PC8-11. */
  gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO8|GPIO9|GPIO10|GPIO11);
  gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO8|GPIO9|GPIO10|GPIO11);
  /* SDIO_CK on PC12. */
  gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12);
  gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO12);
  /* SDIO_CMD on PD2. */
  gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO2);
  gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO2);

  gpio_set_af(GPIOC, GPIO_AF12, GPIO8|GPIO9|GPIO10|GPIO11|GPIO12);
  gpio_set_af(GPIOD, GPIO_AF12, GPIO2);

  SDIO_CLKCR = SDIO_CLKCR_HWFC_EN | SDIO_CLKCR_WIDBUS_1 | (0 << SDIO_CLKCR_CLKDIV_SHIFT);

  nvic_set_priority(NVIC_SDIO_IRQ, 9<<4);
  nvic_enable_irq(NVIC_SDIO_IRQ);

  rcc_periph_clock_enable(DMA_RCC_PERIPH);
  dma_stream_reset(DMA_PERIPH, DMA_STREAM_INSTANCE);
  dma_enable_fifo_mode(DMA_PERIPH, DMA_STREAM_INSTANCE);
  dma_set_fifo_threshold(DMA_PERIPH, DMA_STREAM_INSTANCE, DMA_SxFCR_FTH_4_4_FULL);
  dma_set_memory_burst(DMA_PERIPH, DMA_STREAM_INSTANCE, DMA_SxCR_MBURST_INCR4);
  dma_set_peripheral_burst(DMA_PERIPH, DMA_STREAM_INSTANCE, DMA_SxCR_PBURST_INCR4);
  dma_set_memory_size(DMA_PERIPH, DMA_STREAM_INSTANCE, DMA_SxCR_MSIZE_32BIT);
  dma_set_peripheral_size(DMA_PERIPH, DMA_STREAM_INSTANCE, DMA_SxCR_PSIZE_32BIT);
  dma_enable_memory_increment_mode(DMA_PERIPH, DMA_STREAM_INSTANCE);
  dma_disable_peripheral_increment_mode(DMA_PERIPH, DMA_STREAM_INSTANCE);
  dma_set_priority(DMA_PERIPH, DMA_STREAM_INSTANCE, DMA_SxCR_PL_MEDIUM);
  dma_set_peripheral_address(DMA_PERIPH, DMA_STREAM_INSTANCE, (uint32_t) (&(SDIO_FIFO)));
  dma_channel_select(DMA_PERIPH, DMA_STREAM_INSTANCE, DMA_SxCR_CHSEL_4);
  dma_set_transfer_mode(DMA_PERIPH, DMA_STREAM_INSTANCE, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
  dma_set_peripheral_flow_control(DMA_PERIPH, DMA_STREAM_INSTANCE);

  nvic_set_priority(DMA_IRQ, 10<<4);
  nvic_enable_irq(DMA_IRQ);
}


static uint32_t
is_sdcard_present(void)
{
  return !gpio_get(SD_DETECT_GPIO, SD_DETECT_PIN);
}


static const char *
sdio_error_name(HAL_SD_ErrorTypedef err)
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


static uint32_t sd_buf[512/sizeof(uint32_t)];
static struct ev_file_status sd_status;
static uint32_t file_sofar = 0;
static uint32_t file_length = 0;
static uint32_t range_start_sec = 0;
static uint32_t range_end_sec = 0;
static uint32_t cached_sector = 0;
static uint8_t have_cached_sector = 0;


static SD_HandleTypeDef my_hsd;
static HAL_SD_CardInfoTypedef my_cardinfo;

static int
handle_stream_bytes(void)
{
  uint32_t sec = sd_status.st_stream_bytes.sec;
  uint32_t i, offset, len;
  HAL_SD_ErrorTypedef err;

  if (!have_cached_sector || cached_sector != sec)
  {
    /* Need to read the requested sector. */
    led_on();
    if ((err = HAL_SD_ReadBlocks_DMA(&my_hsd, sd_buf, (uint64_t)sec*512, 512, 1)) != SD_OK)
    {
      led_off();
      serial_puts("SD read error: ");
      serial_puts(sdio_error_name(err));
      serial_puts("\r\n");
      return 1;
    }
    if ((err = HAL_SD_CheckReadOperation(&my_hsd, 10000000)) != SD_OK)
    {
      led_off();
      serial_puts("SD wait single not ok: ");
      serial_puts(sdio_error_name(err));
      serial_puts("\r\n");
      return 1;
    }
    if (HAL_SD_GetStatus(&my_hsd) != SD_TRANSFER_OK)
    {
      led_off();
      serial_puts("HAL_SD_GetStatus() not ok\r\n");
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
  HAL_SD_ErrorTypedef err;
  int res;

  have_cached_sector = 0;                    /* In case of switch out card. */
  if (!is_sdcard_present())
  {
    serial_puts("No SD card present\r\n");
    return 1;
  }
  err = HAL_SD_Init(&my_hsd, &my_cardinfo,
                    DMA_PERIPH, DMA_STREAM_INSTANCE, DMA_STREAM_INSTANCE);
  if (err != SD_OK)
  {
    serial_puts("HAL_SD_Init() failed: ");
    serial_puts(sdio_error_name(err));
    serial_puts("\r\n");
    return 1;
  }
  err = HAL_SD_WideBusOperation_Config(&my_hsd, SDIO_CLKCR_WIDBUS_4);
  if(err != SD_OK)
  {
    serial_puts("HAL_SD_WideBusOperation_Config() failed: ");
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
  HAL_SD_ErrorTypedef err;

  led_on();
  err = HAL_SD_ReadBlocks_DMA(&my_hsd, *dest, (uint64_t)start*512, 512, count);
  if (err != SD_OK)
  {
    led_off();
    serial_puts("SD read error: ");
    serial_puts(sdio_error_name(err));
    serial_puts("\r\n");
    return 1;
  }
  if ((err = HAL_SD_CheckReadOperation(&my_hsd, 10000000)) != SD_OK)
  {
    led_off();
    serial_puts("SD wait not ok: ");
    serial_puts(sdio_error_name(err));
    serial_puts("\r\n");
    return 1;
  }
  if (HAL_SD_GetStatus(&my_hsd) != SD_TRANSFER_OK)
  {
    led_off();
    serial_puts("HAL_SD_GetStatus() not ok\r\n");
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
  setup_sd_gpio_interrupts_dma();
}
