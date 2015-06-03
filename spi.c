#include "pov3d.h"


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
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

  SPI_Cmd(SPI1, DISABLE);
  SPI_Cmd(SPI2, DISABLE);
  SPI_Cmd(SPI3, DISABLE);

  /* GPIOA Configuration: SPI1 CLK on PA5, MISO on PA6, MOSI on PA7. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

  /* First, setup direct GPIO with lines low. */
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA, GPIO_Pin_5|GPIO_Pin_7);
  /* Then, setup the pins as the real alternate-function SPIs. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /*
    GPIOB Configuration: SPI2 CLK on PB13, MISO on PB14, MOSI on PB15.
    SPI3 CLK on PB3, MISO on PB4, MOSI on PB5.
  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_13|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

  /* First, setup direct GPIO with lines low. */
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_13|GPIO_Pin_15);
  /* Then, setup the pins as the real alternate-function SPIs. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|
                                GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* GPIOA Configuration: LAT1 on PA4. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* Latch is active high; initialise it low. */
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);

  /* GPIOC Configuration: LAT2 on PC6, LAT3 on PC5. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  /* Latch is active high; initialise it low. */
  GPIO_ResetBits(GPIOC, GPIO_Pin_5|GPIO_Pin_6);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  /* SCLK is idle low, both setup and sample on rising edge. */
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  /* SPI1 is on the 84 MHz APB2, so prescalar 4 is 21 MHz. */
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_Init(SPI1, &SPI_InitStructure);
  SPI_Cmd(SPI1, ENABLE);
  /* SPI2/3 is on the 42 MHz APB1, so prescalar 2 is 21 MHz. */
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  SPI_Init(SPI2, &SPI_InitStructure);
  SPI_Init(SPI3, &SPI_InitStructure);
  SPI_Cmd(SPI2, ENABLE);
  SPI_Cmd(SPI3, ENABLE);

  /*
    Setup DMA.
    SPI1 uses DMA2 stream 0 ch 3 (Rx) and stream 3 ch 3 (Tx).
    SPI2 uses DMA1 stream 3 ch 0 (Rx) and stream 4 ch 0 (Tx).
    SPI3 uses DMA1 stream 0 ch 0 (Rx) and stream 5 ch 0 (Tx).
  */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  DMA_DeInit(DMA2_Stream0);
  DMA_DeInit(DMA2_Stream3);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  DMA_DeInit(DMA1_Stream3);
  DMA_DeInit(DMA1_Stream4);
  DMA_DeInit(DMA1_Stream0);
  DMA_DeInit(DMA1_Stream5);

  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  /* Configure SPI1 TX DMA */
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI1->DR));
  DMA_InitStructure.DMA_Channel = DMA_Channel_3;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  DMA_Init(DMA2_Stream3, &DMA_InitStructure);
  /* Configure SPI1 RX DMA */
  DMA_InitStructure.DMA_Channel = DMA_Channel_3;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
  DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  /* Configure SPI2 TX DMA */
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI2->DR));
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);
  /* Configure SPI2 RX DMA */
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
  DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);
  /* Configure SPI3 TX DMA */
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI3->DR));
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);
  /* Configure SPI3 RX DMA */
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
  DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  DMA_Init(DMA1_Stream0, &DMA_InitStructure);
}


static void
tlc1_latch(void)
{
  delay(1);
  GPIO_SetBits(GPIOA, GPIO_Pin_4);
  delay(1);
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);
  delay(1);
}


static void
tlc2_latch(void)
{
  delay(1);
  GPIO_SetBits(GPIOC, GPIO_Pin_6);
  delay(1);
  GPIO_ResetBits(GPIOC, GPIO_Pin_6);
  delay(1);
}


static void
tlc3_latch(void)
{
  delay(1);
  GPIO_SetBits(GPIOC, GPIO_Pin_5);
  delay(1);
  GPIO_ResetBits(GPIOC, GPIO_Pin_5);
  delay(1);
}


static void
dma_to_tlc1(uint8_t *outbuf, uint8_t *inbuf, uint32_t len)
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
dma_to_tlc2(uint8_t *outbuf, uint8_t *inbuf, uint32_t len)
{
  DMA1_Stream3->M0AR = (uint32_t)inbuf;
  DMA1_Stream3->NDTR = len;
  DMA1_Stream4->M0AR = (uint32_t)outbuf;
  DMA1_Stream4->NDTR = len;

  DMA_Cmd(DMA1_Stream3, ENABLE);
  DMA_Cmd(DMA1_Stream4, ENABLE);
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
  while (DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4) == RESET)
    ;
  while (DMA_GetFlagStatus(DMA1_Stream3, DMA_FLAG_TCIF3)==RESET)
    ;
  DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
  DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
  DMA_Cmd(DMA1_Stream4, DISABLE);
  DMA_Cmd(DMA1_Stream3, DISABLE);
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, DISABLE);
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, DISABLE);

  while (!(SPI2->SR & SPI_I2S_FLAG_TXE))
    ;
  while (SPI2->SR & SPI_I2S_FLAG_BSY)
    ;
}


static void
dma_to_tlc3(uint8_t *outbuf, uint8_t *inbuf, uint32_t len)
{
  DMA1_Stream0->M0AR = (uint32_t)inbuf;
  DMA1_Stream0->NDTR = len;
  DMA1_Stream5->M0AR = (uint32_t)outbuf;
  DMA1_Stream5->NDTR = len;

  DMA_Cmd(DMA1_Stream0, ENABLE);
  DMA_Cmd(DMA1_Stream5, ENABLE);
  SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);
  SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
  while (DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG_TCIF5) == RESET)
    ;
  while (DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_TCIF0)==RESET)
    ;
  DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
  DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
  DMA_Cmd(DMA1_Stream5, DISABLE);
  DMA_Cmd(DMA1_Stream0, DISABLE);
  SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, DISABLE);
  SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, DISABLE);

  while (!(SPI3->SR & SPI_I2S_FLAG_TXE))
    ;
  while (SPI3->SR & SPI_I2S_FLAG_BSY)
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


static void
fill_tlc5955_control_latch(uint8_t *buf,
                           uint32_t dc_val, uint32_t bc_val, uint32_t mc_val)
{
  uint32_t pos = 0;
  uint32_t i;

  memset(buf, 0, 97);
  for (i = 0; i < 48; ++i)
    add_bits(buf, 97, dc_val, 7, &pos);
  for (i = 0; i < 3; ++i)
    add_bits(buf, 97, mc_val, 3, &pos);
  for (i = 0; i < 3; ++i)
    add_bits(buf, 97, bc_val, 7, &pos);
  /*
    Control bits:
      366 DSPRPT  1   auto repeat (ToDo: turn off...)
      367 TMGRST  1   display timing reset mode enabled (reset GS at LAT)
      368 RFRESH  0   auto data refresh disabled
      369 ESPWM   0   disable enhanced spectrum PWM
      370 LSDVLT  0   LSD voltage is VCC * 0.7
  */
  add_bits(buf, 97, 0x0b, 5, &pos);
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
setup_tlc5955(void (*latch_func)(void),
              void (*dma_func)(uint8_t *, uint8_t *, uint32_t))
{
  uint8_t databuf[97], inbuf[97], tmpbuf[97];

  /*
    MC=4 is 19.1 mA max.
    BC=0 is 10% -> 1.91 mA.
    DC=0 is 26.2% ->0.5 mA.
    Should be safe even for USB usage.
  */
  fill_tlc5955_control_latch(databuf, 0, 0, 4);
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

  setup_tlc5955(tlc1_latch, dma_to_tlc1);
  setup_tlc5955(tlc2_latch, dma_to_tlc2);
  setup_tlc5955(tlc3_latch, dma_to_tlc3);
}


/*
  TLC data is passed as uint32_t *, as we want them to be 32-bit aligned
  for DMA burst transfers.

  The scanplanes should have a leading 0x0000 word (only the one bit is needed
  to mark GS data, but 16 using bits makes all GS values 16-bit aligned.
*/
void
start_dma_scanplanes(uint32_t *p1, uint32_t *p2, uint32_t *p3)
{
  static const uint32_t len = 2+48*2;           /* 48 outputs + extra word */
  DMA2_Stream3->M0AR = (uint32_t)p1;
  DMA2_Stream3->NDTR = len;
  DMA1_Stream4->M0AR = (uint32_t)p2;
  DMA1_Stream4->NDTR = len;
  DMA1_Stream5->M0AR = (uint32_t)p3;
  DMA1_Stream5->NDTR = len;

  DMA_Cmd(DMA2_Stream3, ENABLE);
  DMA_Cmd(DMA1_Stream4, ENABLE);
  DMA_Cmd(DMA1_Stream5, ENABLE);
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);

  /* ToDo: For now, we wait (just testing). Later we will only start. */
  while (DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET)
    ;
  while (DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4) == RESET)
    ;
  while (DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG_TCIF5) == RESET)
    ;
  DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
  DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
  DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
  DMA_Cmd(DMA2_Stream3, DISABLE);
  DMA_Cmd(DMA1_Stream4, DISABLE);
  DMA_Cmd(DMA1_Stream5, DISABLE);
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, DISABLE);
  SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, DISABLE);

  while (!(SPI1->SR & SPI_I2S_FLAG_TXE))
    ;
  while (!(SPI2->SR & SPI_I2S_FLAG_TXE))
    ;
  while (!(SPI3->SR & SPI_I2S_FLAG_TXE))
    ;
  while (SPI1->SR & SPI_I2S_FLAG_BSY)
    ;
  while (SPI2->SR & SPI_I2S_FLAG_BSY)
    ;
  while (SPI3->SR & SPI_I2S_FLAG_BSY)
    ;
  /* Latch. */
  delay(1);
  GPIO_SetBits(GPIOA, GPIO_Pin_4);
  GPIO_SetBits(GPIOC, GPIO_Pin_6);
  GPIO_SetBits(GPIOC, GPIO_Pin_5);
  delay(1);
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);
  GPIO_ResetBits(GPIOC, GPIO_Pin_6);
  GPIO_ResetBits(GPIOC, GPIO_Pin_5);
}
