#include "ledtorus.h"

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/dma.h>


#define TLC_NUM 3

/*
  TLCs are on:
    PA5 (CLK) PA6 (MISO) PA7 (MOSI) PA4 (LAT)
    PA3 is VPRG/MODE.

    PB13 (CLK) PB14 (MISO) PB15 (MOSI) PC6 (LAT)

    PB3 (CLK) PB4 (MISO) PB5 (MOSI) PC5 (LAT)
*/


static void
setup_tlc_spi_dma()
{
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_SPI1);
  rcc_periph_clock_enable(RCC_SPI2);
  rcc_periph_clock_enable(RCC_SPI3);

  spi_disable(SPI1);
  spi_disable(SPI2);
  spi_disable(SPI3);

  /* GPIOA Configuration: SPI1 CLK on PA5, MISO on PA6, MOSI on PA7. */
  gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO6);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO6);
  /* First, setup direct GPIO with lines low. */
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, GPIO5|GPIO7);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO5|GPIO7);
  gpio_clear(GPIOA, GPIO5|GPIO7);
  /* Then, setup the pins as the real alternate-function SPIs. */
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO5|GPIO6|GPIO7);

  /*
    GPIOB Configuration: SPI2 CLK on PB13, MISO on PB14, MOSI on PB15.
    SPI3 CLK on PB3, MISO on PB4, MOSI on PB5.
  */
  gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO4|GPIO14);
  gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO4|GPIO14);
  /* First, setup direct GPIO with lines low. */
  gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN,
                  GPIO3|GPIO5|GPIO13|GPIO15);
  gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ,
                          GPIO3|GPIO5|GPIO13|GPIO15);
  gpio_clear(GPIOB, GPIO3|GPIO5|GPIO13|GPIO15);
  /* Then, setup the pins as the real alternate-function SPIs. */
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN,
                  GPIO3|GPIO4|GPIO5|GPIO13|GPIO14|GPIO15);

  /* GPIOA Configuration: LAT1 on PA4. */
  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO4);
  /* Latch is active high; initialise it low. */
  gpio_clear(GPIOA, GPIO4);

  /* GPIOC Configuration: LAT2 on PC6, LAT3 on PC5. */
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5|GPIO6);
  gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO5|GPIO6);
  /* Latch is active high; initialise it low. */
  gpio_clear(GPIOA, GPIO5|GPIO6);

  /* SPI1 is on alternate function 5. */
  gpio_set_af(GPIOA, GPIO_AF5, GPIO5);
  gpio_set_af(GPIOA, GPIO_AF5, GPIO6);
  gpio_set_af(GPIOA, GPIO_AF5, GPIO7);
  /* SPI2 is on alternate function 5. */
  gpio_set_af(GPIOB, GPIO_AF5, GPIO13);
  gpio_set_af(GPIOB, GPIO_AF5, GPIO14);
  gpio_set_af(GPIOB, GPIO_AF5, GPIO15);
  /* SPI3 is on alternate function 6. */
  gpio_set_af(GPIOB, GPIO_AF6, GPIO3);
  gpio_set_af(GPIOB, GPIO_AF6, GPIO4);
  gpio_set_af(GPIOB, GPIO_AF6, GPIO5);

  spi_init_master(SPI1,
                  /* SPI1 is on the 84 MHz APB2, so prescalar 4 is 21 MHz. */
                  SPI_CR1_BAUDRATE_FPCLK_DIV_4,
                  /* SCLK is idle low, both setup and sample on rising edge. */
                  SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_1,
                  SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_set_unidirectional_mode(SPI1);
  spi_set_full_duplex_mode(SPI1);
  spi_enable_software_slave_management(SPI1);
  spi_set_nss_high(SPI1);
  spi_enable(SPI1);

  spi_init_master(SPI2,
                  /* SPI2 is on the 42 MHz APB1, so prescalar 2 is 21 MHz. */
                  SPI_CR1_BAUDRATE_FPCLK_DIV_2,
                  /* SCLK is idle low, both setup and sample on rising edge. */
                  SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_1,
                  SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_set_unidirectional_mode(SPI2);
  spi_set_full_duplex_mode(SPI2);
  spi_enable_software_slave_management(SPI2);
  spi_set_nss_high(SPI2);
  spi_enable(SPI2);

  spi_init_master(SPI3,
                  /* SPI3 is on the 42 MHz APB1, so prescalar 2 is 21 MHz. */
                  SPI_CR1_BAUDRATE_FPCLK_DIV_2,
                  /* SCLK is idle low, both setup and sample on rising edge. */
                  SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_1,
                  SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_set_unidirectional_mode(SPI3);
  spi_set_full_duplex_mode(SPI3);
  spi_enable_software_slave_management(SPI3);
  spi_set_nss_high(SPI3);
  spi_enable(SPI3);

  /*
    Setup DMA.
    SPI1 uses DMA2 stream 0 ch 3 (Rx) and stream 3 ch 3 (Tx).
    SPI2 uses DMA1 stream 3 ch 0 (Rx) and stream 4 ch 0 (Tx).
    SPI3 uses DMA1 stream 0 ch 0 (Rx) and stream 5 ch 0 (Tx).
  */
  rcc_periph_clock_enable(RCC_DMA2);
  dma_stream_reset(DMA2, DMA_STREAM0);
  dma_stream_reset(DMA2, DMA_STREAM3);
  rcc_periph_clock_enable(RCC_DMA1);
  dma_stream_reset(DMA1, DMA_STREAM3);
  dma_stream_reset(DMA1, DMA_STREAM4);
  dma_stream_reset(DMA1, DMA_STREAM0);
  dma_stream_reset(DMA1, DMA_STREAM5);

  /* Configure SPI1 TX DMA */
  dma_enable_direct_mode(DMA2, DMA_STREAM3);
  dma_set_fifo_threshold(DMA2, DMA_STREAM3, DMA_SxFCR_FTH_1_4_FULL);
  dma_set_memory_burst(DMA2, DMA_STREAM3, DMA_SxCR_MBURST_SINGLE);
  dma_set_peripheral_burst(DMA2, DMA_STREAM3, DMA_SxCR_PBURST_SINGLE);
  dma_set_memory_size(DMA2, DMA_STREAM3, DMA_SxCR_MSIZE_8BIT);
  dma_set_peripheral_size(DMA2, DMA_STREAM3, DMA_SxCR_PSIZE_8BIT);
  dma_enable_memory_increment_mode(DMA2, DMA_STREAM3);
  dma_disable_peripheral_increment_mode(DMA2, DMA_STREAM3);
  dma_set_priority(DMA2, DMA_STREAM3, DMA_SxCR_PL_HIGH);
  dma_set_peripheral_address(DMA2, DMA_STREAM3, (uint32_t) (&(SPI1_DR)));
  dma_channel_select(DMA2, DMA_STREAM3, DMA_SxCR_CHSEL_3);
  dma_set_transfer_mode(DMA2, DMA_STREAM3, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
  /* Configure SPI1 RX DMA */
  dma_enable_direct_mode(DMA2, DMA_STREAM0);
  dma_set_fifo_threshold(DMA2, DMA_STREAM0, DMA_SxFCR_FTH_1_4_FULL);
  dma_set_memory_burst(DMA2, DMA_STREAM0, DMA_SxCR_MBURST_SINGLE);
  dma_set_peripheral_burst(DMA2, DMA_STREAM0, DMA_SxCR_PBURST_SINGLE);
  dma_set_memory_size(DMA2, DMA_STREAM0, DMA_SxCR_MSIZE_8BIT);
  dma_set_peripheral_size(DMA2, DMA_STREAM0, DMA_SxCR_PSIZE_8BIT);
  dma_enable_memory_increment_mode(DMA2, DMA_STREAM0);
  dma_disable_peripheral_increment_mode(DMA2, DMA_STREAM0);
  dma_set_priority(DMA2, DMA_STREAM0, DMA_SxCR_PL_HIGH);
  dma_set_peripheral_address(DMA2, DMA_STREAM0, (uint32_t) (&(SPI1_DR)));
  dma_channel_select(DMA2, DMA_STREAM0, DMA_SxCR_CHSEL_3);
  dma_set_transfer_mode(DMA2, DMA_STREAM0, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);

  /* Configure SPI2 TX DMA */
  dma_enable_direct_mode(DMA1, DMA_STREAM4);
  dma_set_fifo_threshold(DMA1, DMA_STREAM4, DMA_SxFCR_FTH_1_4_FULL);
  dma_set_memory_burst(DMA1, DMA_STREAM4, DMA_SxCR_MBURST_SINGLE);
  dma_set_peripheral_burst(DMA1, DMA_STREAM4, DMA_SxCR_PBURST_SINGLE);
  dma_set_memory_size(DMA1, DMA_STREAM4, DMA_SxCR_MSIZE_8BIT);
  dma_set_peripheral_size(DMA1, DMA_STREAM4, DMA_SxCR_PSIZE_8BIT);
  dma_enable_memory_increment_mode(DMA1, DMA_STREAM4);
  dma_disable_peripheral_increment_mode(DMA1, DMA_STREAM4);
  dma_set_priority(DMA1, DMA_STREAM4, DMA_SxCR_PL_HIGH);
  dma_set_peripheral_address(DMA1, DMA_STREAM4, (uint32_t) (&(SPI2_DR)));
  dma_channel_select(DMA1, DMA_STREAM4, DMA_SxCR_CHSEL_0);
  dma_set_transfer_mode(DMA1, DMA_STREAM4, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);

  /* Configure SPI2 RX DMA */
  dma_enable_direct_mode(DMA1, DMA_STREAM3);
  dma_set_fifo_threshold(DMA1, DMA_STREAM3, DMA_SxFCR_FTH_1_4_FULL);
  dma_set_memory_burst(DMA1, DMA_STREAM3, DMA_SxCR_MBURST_SINGLE);
  dma_set_peripheral_burst(DMA1, DMA_STREAM3, DMA_SxCR_PBURST_SINGLE);
  dma_set_memory_size(DMA1, DMA_STREAM3, DMA_SxCR_MSIZE_8BIT);
  dma_set_peripheral_size(DMA1, DMA_STREAM3, DMA_SxCR_PSIZE_8BIT);
  dma_enable_memory_increment_mode(DMA1, DMA_STREAM3);
  dma_disable_peripheral_increment_mode(DMA1, DMA_STREAM3);
  dma_set_priority(DMA1, DMA_STREAM3, DMA_SxCR_PL_HIGH);
  dma_set_peripheral_address(DMA1, DMA_STREAM3, (uint32_t) (&(SPI2_DR)));
  dma_channel_select(DMA1, DMA_STREAM3, DMA_SxCR_CHSEL_0);
  dma_set_transfer_mode(DMA1, DMA_STREAM3, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);

  /* Configure SPI3 TX DMA */
  dma_enable_direct_mode(DMA1, DMA_STREAM5);
  dma_set_fifo_threshold(DMA1, DMA_STREAM5, DMA_SxFCR_FTH_1_4_FULL);
  dma_set_memory_burst(DMA1, DMA_STREAM5, DMA_SxCR_MBURST_SINGLE);
  dma_set_peripheral_burst(DMA1, DMA_STREAM5, DMA_SxCR_PBURST_SINGLE);
  dma_set_memory_size(DMA1, DMA_STREAM5, DMA_SxCR_MSIZE_8BIT);
  dma_set_peripheral_size(DMA1, DMA_STREAM5, DMA_SxCR_PSIZE_8BIT);
  dma_enable_memory_increment_mode(DMA1, DMA_STREAM5);
  dma_disable_peripheral_increment_mode(DMA1, DMA_STREAM5);
  dma_set_priority(DMA1, DMA_STREAM5, DMA_SxCR_PL_HIGH);
  dma_set_peripheral_address(DMA1, DMA_STREAM5, (uint32_t) (&(SPI3_DR)));
  dma_channel_select(DMA1, DMA_STREAM5, DMA_SxCR_CHSEL_0);
  dma_set_transfer_mode(DMA1, DMA_STREAM5, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);

  /* Configure SPI3 RX DMA */
  dma_enable_direct_mode(DMA1, DMA_STREAM0);
  dma_set_fifo_threshold(DMA1, DMA_STREAM0, DMA_SxFCR_FTH_1_4_FULL);
  dma_set_memory_burst(DMA1, DMA_STREAM0, DMA_SxCR_MBURST_SINGLE);
  dma_set_peripheral_burst(DMA1, DMA_STREAM0, DMA_SxCR_PBURST_SINGLE);
  dma_set_memory_size(DMA1, DMA_STREAM0, DMA_SxCR_MSIZE_8BIT);
  dma_set_peripheral_size(DMA1, DMA_STREAM0, DMA_SxCR_PSIZE_8BIT);
  dma_enable_memory_increment_mode(DMA1, DMA_STREAM0);
  dma_disable_peripheral_increment_mode(DMA1, DMA_STREAM0);
  dma_set_priority(DMA1, DMA_STREAM0, DMA_SxCR_PL_HIGH);
  dma_set_peripheral_address(DMA1, DMA_STREAM0, (uint32_t) (&(SPI3_DR)));
  dma_channel_select(DMA1, DMA_STREAM0, DMA_SxCR_CHSEL_0);
  dma_set_transfer_mode(DMA1, DMA_STREAM0, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
}


static void
tlc1_latch(void)
{
  delay(1);
  gpio_set(GPIOA, GPIO4);
  delay(1);
  gpio_clear(GPIOA, GPIO4);
  delay(1);
}


static void
tlc2_latch(void)
{
  delay(1);
  gpio_set(GPIOC, GPIO6);
  delay(1);
  gpio_clear(GPIOC, GPIO6);
  delay(1);
}


static void
tlc3_latch(void)
{
  delay(1);
  gpio_set(GPIOC, GPIO5);
  delay(1);
  gpio_clear(GPIOC, GPIO5);
  delay(1);
}


static void
dma_to_tlc1(uint8_t *outbuf, uint8_t *inbuf, uint32_t len)
{
  DMA2_S0M0AR = inbuf;
  DMA2_S0NDTR = len;
  DMA2_S3M0AR = outbuf;
  DMA2_S3NDTR = len;

  dma_enable_stream(DMA2, DMA_STREAM0);
  dma_enable_stream(DMA2, DMA_STREAM3);
  spi_enable_rx_dma(SPI1);
  spi_enable_tx_dma(SPI1);
  while (!dma_get_interrupt_flag(DMA2, DMA_STREAM3, DMA_TCIF))
    ;
  while (!dma_get_interrupt_flag(DMA2, DMA_STREAM0, DMA_TCIF))
    ;
  dma_clear_interrupt_flags(DMA2, DMA_STREAM3, DMA_TCIF);
  dma_clear_interrupt_flags(DMA2, DMA_STREAM0, DMA_TCIF);
  dma_disable_stream(DMA2, DMA_STREAM3);
  dma_disable_stream(DMA2, DMA_STREAM0);
  spi_disable_rx_dma(SPI1);
  spi_disable_tx_dma(SPI1);

  while (!(SPI1_SR & SPI_SR_TXE))
    ;
  while (SPI1_SR & SPI_SR_BSY)
    ;
}


static void
dma_to_tlc2(uint8_t *outbuf, uint8_t *inbuf, uint32_t len)
{
  DMA1_S3M0AR = inbuf;
  DMA1_S3NDTR = len;
  DMA1_S4M0AR = outbuf;
  DMA1_S4NDTR = len;

  dma_enable_stream(DMA1, DMA_STREAM3);
  dma_enable_stream(DMA1, DMA_STREAM4);
  spi_enable_rx_dma(SPI2);
  spi_enable_tx_dma(SPI2);
  while (!dma_get_interrupt_flag(DMA1, DMA_STREAM4, DMA_TCIF))
    ;
  while (!dma_get_interrupt_flag(DMA1, DMA_STREAM3, DMA_TCIF))
    ;
  dma_clear_interrupt_flags(DMA1, DMA_STREAM4, DMA_TCIF);
  dma_clear_interrupt_flags(DMA1, DMA_STREAM3, DMA_TCIF);
  dma_disable_stream(DMA1, DMA_STREAM4);
  dma_disable_stream(DMA1, DMA_STREAM3);
  spi_disable_rx_dma(SPI2);
  spi_disable_tx_dma(SPI2);

  while (!(SPI2_SR & SPI_SR_TXE))
    ;
  while (SPI2_SR & SPI_SR_BSY)
    ;
}


static void
dma_to_tlc3(uint8_t *outbuf, uint8_t *inbuf, uint32_t len)
{
  DMA1_S0M0AR = inbuf;
  DMA1_S0NDTR = len;
  DMA1_S5M0AR = outbuf;
  DMA1_S5NDTR = len;

  dma_enable_stream(DMA1, DMA_STREAM0);
  dma_enable_stream(DMA1, DMA_STREAM5);
  spi_enable_rx_dma(SPI3);
  spi_enable_tx_dma(SPI3);
  while (!dma_get_interrupt_flag(DMA1, DMA_STREAM5, DMA_TCIF))
    ;
  while (!dma_get_interrupt_flag(DMA1, DMA_STREAM0, DMA_TCIF))
    ;
  dma_clear_interrupt_flags(DMA1, DMA_STREAM5, DMA_TCIF);
  dma_clear_interrupt_flags(DMA1, DMA_STREAM0, DMA_TCIF);
  dma_disable_stream(DMA1, DMA_STREAM5);
  dma_disable_stream(DMA1, DMA_STREAM0);
  spi_disable_rx_dma(SPI3);
  spi_disable_tx_dma(SPI3);

  while (!(SPI3_SR & SPI_SR_TXE))
    ;
  while (SPI3_SR & SPI_SR_BSY)
    ;
}


static void
add_bits(uint8_t *buf, uint32_t len, uint32_t val, uint32_t size, uint32_t *pos)
{
  uint32_t loc_pos = *pos;
  while (size)
  {
    if (val & 1)
      buf[len - 1 - (loc_pos>>3)] |= 1 << (loc_pos & 7);
    val >>= 1;
    --size;
    ++loc_pos;
  }
  *pos = loc_pos;
}


static void
shift_buf_7_bits(uint8_t *buf, uint32_t len)
{
  uint32_t i;
  uint8_t old;

  old = 0;
  for (i = 0; i < len; ++i)
  {
    uint8_t val = buf[i];
    buf[i] = (old << 1) | (val >> 7);
    old = val;
  }
}


/*
  Some possible MC/BC combinations:

    MC=2, BC=0 -> 1.12 mA max.
    TLC5955 datasheet says that current at max. DC should not be lower than
    1 mA, so this should be minimal configuration.

    MC=4, BC=0..127 -> 1.91 .. 19.1 mA.
*/
void
fill_tlc5955_control_latch(uint8_t *buf,
                           uint32_t tlc_idx, uint32_t bc_val, uint32_t mc_val)
{
  uint32_t pos = 0;
  uint32_t i;
  float max_dist = led_distance_to_center_xy(6,3);

  memset(buf, 0, 97);
  for (i = 0; i < 48; ++i)
  {
    uint32_t led = i/3;
    uint32_t dc_adj;
    float dist = led_distance_to_center_tlc(tlc_idx, led);
    float dc_fact = dist / max_dist;
    /* Minimum brightness @ DC=0 is 26.2%. */
    if (dc_fact < 0.262f)
      dc_fact = 0.262f;
    dc_adj = (uint32_t)(0.5f + 127.0f * (dc_fact - 0.262f)/(1.0f - 0.262f));
    add_bits(buf, 97, dc_adj, 7, &pos);
  }
  for (i = 0; i < 3; ++i)
    add_bits(buf, 97, mc_val, 3, &pos);
  for (i = 0; i < 3; ++i)
    add_bits(buf, 97, bc_val, 7, &pos);
  /*
    Control bits:
      366 DSPRPT  0   Disable auto repeat
      367 TMGRST  1   display timing reset mode enabled (reset GS at LAT)
      368 RFRESH  0   auto data refresh disabled
      369 ESPWM   1   Enable enhanced spectrum PWM
      370 LSDVLT  0   LSD voltage is VCC * 0.7
  */
  add_bits(buf, 97, 0x0a, 5, &pos);
  /* Need 0x1 0x96 at start of buffer to latch control register. */
  pos = 760;
  add_bits(buf, 97, 0x196, 9, &pos);
}


static void
fill_tlc5955_gs_latch(uint8_t *buf, uint32_t max_gs)
{
  uint32_t i, j;
  uint32_t idx;

  /*
    Bit 768=0 latches GS register. We shift out bits 769-775 also, for
    simplicity.
  */
  buf[0] = 0;
  idx = 1;
  for (i = 0; i < 16; ++i)
  {
    for (j = 0; j < 3; ++j)
    {
      uint32_t gsval = ( (i & (1<<j)) ? max_gs/(2-(i/8)) : 0);
      buf[idx++] = gsval >> 8;
      buf[idx++] = gsval & 0xff;
    }
  }
}


static void
setup_tlc5955(uint32_t tlc_idx, void (*latch_func)(void),
              void (*dma_func)(uint8_t *, uint8_t *, uint32_t))
{
  uint8_t databuf[97], inbuf[97], tmpbuf[97];

  /*
    MC=4 is 19.1 mA max.
    BC=0 is 10% -> 1.91 mA.
    BC=127 is 100% -> 19.1 mA.
    Should be safe even for USB usage, or can use MC=2 BC=0 for 1.12 mA.
  */
  fill_tlc5955_control_latch(databuf, tlc_idx, led_intensity, 4);
  serial_puts("Sending control register data to TLC5955:\r\n");
  serial_dump_buf(databuf, sizeof(databuf));
  (*dma_func)(databuf, inbuf, sizeof(databuf));
  (*latch_func)();

  memset(tmpbuf, 0x00, sizeof(tmpbuf));
  (*dma_func)(tmpbuf, inbuf, sizeof(tmpbuf));
  (*latch_func)();
  /* Since we shift out 7 bits too many, we need to adjust the result read. */
  shift_buf_7_bits(inbuf, 97);
  if (0 != memcmp(databuf, inbuf, sizeof(databuf)))
  {
    serial_dump_buf(inbuf, sizeof(inbuf));
    serial_puts(" ouch, read control data does not match!\r\n");
    for (;;)
      ;
  }
  else
    serial_puts("Control data verified ok, rewrite to confirm config\r\n");
  (*dma_func)(databuf, inbuf, sizeof(databuf));
  (*latch_func)();
  serial_puts("Now load some GS data ...\r\n");
  fill_tlc5955_gs_latch(tmpbuf, 4095);
  serial_dump_buf(tmpbuf, sizeof(tmpbuf));
  (*dma_func)(tmpbuf, inbuf, sizeof(tmpbuf));
  (*latch_func)();
}


void
setup_spi(void)
{
  setup_tlc_spi_dma();
  /* ToDo: Setup nRF. */

  setup_tlc5955(0, tlc1_latch, dma_to_tlc1);
  setup_tlc5955(1, tlc2_latch, dma_to_tlc2);
  setup_tlc5955(2, tlc3_latch, dma_to_tlc3);
}


/*
  Start transfer of scanplanes to the TLC5955 chips.

  The scanplanes are each a sequence of 48 16-bit PWM grayscale values in
  big-endian format. The scanplanes should have a leading 0x0000 word (only
  the one bit is needed to mark GS data, but 16 using bits makes all GS values
  16-bit aligned.

  This function only starts the DMA transfer. After the transfer is complete,
  the latch_scanplanes() function must be called to actually latch the GS
  values into the grayscale registers.

  The end of the transfer can be queried using is_tlc_dma_done().

  TLC data is passed as uint32_t *, as we want them to be 32-bit aligned
  for DMA burst transfers.
*/
void
start_dma_scanplanes(uint32_t *p1, uint32_t *p2, uint32_t *p3)
{
  static const uint32_t len = 2+48*2;           /* 48 outputs + extra word */

  dma_clear_interrupt_flags(DMA2, DMA_STREAM3, DMA_TCIF);
  dma_clear_interrupt_flags(DMA1, DMA_STREAM4, DMA_TCIF);
  dma_clear_interrupt_flags(DMA1, DMA_STREAM5, DMA_TCIF);

  DMA2_S3M0AR = p1;
  DMA2_S3NDTR = len;
  DMA1_S4M0AR = p2;
  DMA1_S4NDTR = len;
  DMA1_S5M0AR = p3;
  DMA1_S5NDTR = len;

  dma_enable_stream(DMA2, DMA_STREAM3);
  dma_enable_stream(DMA1, DMA_STREAM4);
  dma_enable_stream(DMA1, DMA_STREAM5);
  spi_enable_tx_dma(SPI1);
  spi_enable_tx_dma(SPI2);
  spi_enable_tx_dma(SPI3);
}


void
latch_scanplanes(void)
{
  delay(1);
  gpio_set(GPIOA, GPIO4);
  gpio_set(GPIOA, GPIO6);
  gpio_set(GPIOA, GPIO5);
  delay(1);
  gpio_clear(GPIOA, GPIO4);
  gpio_clear(GPIOA, GPIO6);
  gpio_clear(GPIOA, GPIO5);
}


uint32_t
is_tlc_dma_done(void)
{
  if (!dma_get_interrupt_flag(DMA2, DMA_STREAM3, DMA_TCIF) ||
      !dma_get_interrupt_flag(DMA1, DMA_STREAM4, DMA_TCIF) ||
      !dma_get_interrupt_flag(DMA1, DMA_STREAM5, DMA_TCIF))
    return 0;
  if (!(SPI1_SR & SPI_SR_TXE) ||
      !(SPI2_SR & SPI_SR_TXE) ||
      !(SPI3_SR & SPI_SR_TXE))
    return 0;
  if ((SPI1_SR & SPI_SR_BSY) ||
      (SPI2_SR & SPI_SR_BSY) ||
      (SPI3_SR & SPI_SR_BSY))
    return 0;
  return 1;
}
