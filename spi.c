#include "pov3d.h"


/*
  TLCs are on:
    PA5 (CLK) PA6 (MISO) PA7 (MOSI) PA4 (LAT)
    PA3 is VPRG/MODE.
*/


static void
setup_tlc()
{
  /* ToDo: make the SPI a parameter so can be used for all 3. */

  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  SPI_Cmd(SPI1, DISABLE);
  /* GPIOA Configuration: SPI1 CLK on PA5, MISO on PA6, MOSI on PA7. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

  /* First, setup direct GPIO with lines low. */
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA, GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
  /* Then, setup the pins as the real alternate-function SPIs. */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* GPIOC Configuration: LAT on PA4. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* Latch is active high; initialise it low. */
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  /* SCLK is idle low, both setup and sample on rising edge. */
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = (SPI_CPHA_1Edge);
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
  /* SPI1 is on the 84 MHz APB2, so prescalar 4 is 21 MHz. */
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);
  SPI_Cmd(SPI1, ENABLE);

  /*
    Setup DMA.
    SPI1 uses DMA2 stream 0 ch 3 (Rx) and stream 3 ch 3 (Tx).
  */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  DMA_DeInit(DMA2_Stream0);
  DMA_DeInit(DMA2_Stream3);

  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI1->DR));
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  /* Configure TX DMA */
  DMA_InitStructure.DMA_Channel = DMA_Channel_3;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  DMA_Init(DMA2_Stream3, &DMA_InitStructure);
  /* Configure RX DMA */
  DMA_InitStructure.DMA_Channel = DMA_Channel_3;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
  DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
}


/* ToDo: This is only for TLC5940 testing. */
static void
setup_tcl_vprg(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* GPIOA Configuration: VPRG on PA3. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}


static void
tlc_mode_dc(void)
{
  delay(5);
  GPIO_SetBits(GPIOA, GPIO_Pin_3);
  delay(5);
}


static void
tlc_mode_gs(void)
{
  delay(5);
  GPIO_ResetBits(GPIOA, GPIO_Pin_3);
  delay(5);
}


static void
tlc_latch(void)
{
  delay(1);
  GPIO_SetBits(GPIOA, GPIO_Pin_4);
  delay(1);
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);
  delay(1);
}


/*
  There is a slight oddity with the TLC5940, when reading out the status
  information. The first bit of status is put into the shift register (and thus
  onto SOUT) only at the first SCLK transition _after_ latching grayscale data.
  So we need to pulse SCLK one extra time (effectively shifting out one dummy
  bit) after latching, and before reading out the status information.
*/
static void
tlc_shiftout_extra_bit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Temporarily switch SPI pins to direct GPIO, and manually pulse one bit. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_ResetBits(GPIOA, GPIO_Pin_7); /* SIN low (not that it matters...) */
  GPIO_SetBits(GPIOA, GPIO_Pin_5);   /* SCLK low */
  delay(1);
  GPIO_ResetBits(GPIOA, GPIO_Pin_5); /* SCLK high */

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}


static void __attribute__((unused))
send_to_tlcs(uint8_t *outbuf, uint8_t *inbuf, uint32_t len)
{
  uint32_t i;

  if (!len)
    return;

  while (!(SPI1->SR & SPI_I2S_FLAG_TXE))
    ;
  SPI1->DR = (uint8_t)outbuf[0];
  for (i = 1; i < len; ++i)
  {
    while (!(SPI1->SR & SPI_I2S_FLAG_TXE))
      ;
    SPI1->DR = (uint8_t)outbuf[i];
    while (!(SPI1->SR & SPI_I2S_FLAG_RXNE))
      ;
    inbuf[i-1] = (uint8_t)SPI1->DR;
  }
  while (!(SPI1->SR & SPI_I2S_FLAG_RXNE))
    ;
  inbuf[len-1] = (uint8_t)SPI1->DR;
}


static void
dma_to_tlcs(uint8_t *outbuf, uint8_t *inbuf, uint32_t len)
{
  DMA2_Stream0->M0AR = (uint32_t)inbuf;
  DMA2_Stream0->NDTR = len;
  DMA2_Stream3->M0AR = (uint32_t)outbuf;
  DMA2_Stream3->NDTR = len;

  DMA_Cmd(DMA2_Stream0, ENABLE);
  DMA_Cmd(DMA2_Stream3, ENABLE);
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
  while (DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET)
    ;
  while (DMA_GetFlagStatus(DMA2_Stream0, DMA_FLAG_TCIF0)==RESET)
    ;
  DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
  DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_TCIF0);
  DMA_Cmd(DMA2_Stream3, DISABLE);
  DMA_Cmd(DMA2_Stream0, DISABLE);
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);

  while (!(SPI1->SR & SPI_I2S_FLAG_TXE))
    ;
  while (SPI1->SR & SPI_I2S_FLAG_BSY)
    ;
}


static void
setup_tlc_dc(void)
{
  /*
    ToDo: Change to use TLC5955, once we have hardware.
    For now, this tests using a string of 6 TLC5940s.
  */
  uint8_t dc_buf[6*16*6/8];
  uint8_t status_buf[192*6/8];
  uint32_t i, j, sofar, dc, bit, idx;

  bit = 0;
  sofar = 0;
  idx = 0;
  for (i = 0; i < 6*16; ++i)
  {
    dc = 1;
    for (j = 0; j < 6; ++j)
    {
      sofar <<= 1;
      sofar |= (dc & (1 << (5-j)) ? 1 : 0);
      ++bit;
      if (bit == 8)
      {
        dc_buf[idx] = sofar;
        sofar = 0;
        bit = 0;
        ++idx;
      }
    }
  }

  serial_puts("Sending DC data to TLCs:\r\n");
  serial_dump_buf(dc_buf, sizeof(dc_buf));
  tlc_mode_dc();
  /*
    Send a dummy byte first, will be shifted out at the other end.
    Seems to avoid a problem where we occasionally get the very first bit
    shifted out be a "1" instead of a "0"...
  */
  status_buf[0] = 0;
  dma_to_tlcs(status_buf, status_buf, 1);
  dma_to_tlcs(dc_buf, status_buf, sizeof(dc_buf));
  tlc_latch();
  serial_puts("Data back from sending DC:\r\n");
  serial_dump_buf(status_buf, sizeof(dc_buf));

  serial_puts("Now latch dummy GS data, to put status info in shift register.\r\n");
  memset(status_buf, 0x00, sizeof(status_buf));
  tlc_mode_gs();
  dma_to_tlcs(status_buf, status_buf, sizeof(status_buf));
  tlc_latch();
  serial_puts("(Data back from GS latch:)\r\n");
  serial_dump_buf(status_buf, sizeof(status_buf));
  serial_puts("Now reading back status info from the TLCs...\r\n");
  memset(status_buf, 0x00, sizeof(status_buf));
  tlc_shiftout_extra_bit();
  dma_to_tlcs(status_buf, status_buf, sizeof(status_buf));
  serial_puts("TLC status buffer:\r\n");
  serial_dump_buf(status_buf, sizeof(status_buf));

  serial_puts("Check that DC data is loaded correctly...");
  for (i = 0; i < 6; ++i)
  {
    if (0 != memcmp(dc_buf + i*(16*6/8), status_buf + i*(192/8) + 3, (16*6/8)))
    {
      serial_puts(" ouch, read DC data does not match for TLC number ");
      println_uint32(i);
      for (;;)
        ;
    }
  }
  serial_puts(" all ok!\r\n");
  serial_puts("Now load some GS data ...\r\n");
  idx = 0;
  for (i = 0; i < 16*6/3; ++i)
  {
    for (j = 0; j < 3; ++j)
    {
      uint32_t gsval = ( (i & (1<<j)) ? 4095/(4-(i/8)) : 0);
      if (!((i*3+j) & 1))
      {
        status_buf[idx] = gsval >> 4;
        ++idx;
        status_buf[idx] = (gsval << 4) & 0xf0;
      }
      else
      {
        status_buf[idx] |= (gsval >> 8);
        ++idx;
        status_buf[idx] = (gsval & 0xff);
        ++idx;
      }
    }
  }
  dma_to_tlcs(status_buf, status_buf, sizeof(status_buf));
  tlc_latch();

  serial_puts("Dat's all .oO\r\n");
}


void
setup_spi(void)
{
  setup_tlc();
  setup_tcl_vprg();
  /* ToDo: Setup two remaining TLCs, and nRF. */

  setup_tlc_dc();
}
