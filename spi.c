#include "ledtorus.h"


#define TLC_NUM 3

/*
  TLCs are on:
    SPI1:   U8/U9   PA5 (CLK)  PA6 (MISO)  PA7 (MOSI)  PA4 (LAT)
    SPI2: U12/U13  PB13 (CLK) PB14 (MISO) PB15 (MOSI) PD11 (LAT)
    SPI3:   U6/U7   PB3 (CLK)  PB4 (MISO)  PB5 (MOSI) PG15 (LAT)
    SPI4: U10/U11  PE12 (CLK) PE13 (MISO) PE14 (MOSI) PE15 (LAT)
    SPI5: U14/U15   PH6 (CLK)  PH7 (MISO) PF11 (MOSI) PD10 (LAT)
    SPI6:   U4/U5  PG13 (CLK) PG12 (MISO) PG14 (MOSI)  PH3 (LAT)
*/


static void
setup_tlc_spi_dma()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI6, ENABLE);

  SPI_Cmd(SPI1, DISABLE);
  SPI_Cmd(SPI2, DISABLE);
  SPI_Cmd(SPI3, DISABLE);
  SPI_Cmd(SPI4, DISABLE);
  SPI_Cmd(SPI5, DISABLE);
  SPI_Cmd(SPI6, DISABLE);

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

  /* GPIOE Configuration: SPI4 CLK on PE12, MISO on PE13, MOSI on PE14. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

  /* First, setup direct GPIO with lines low. */
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOE, GPIO_Pin_12|GPIO_Pin_14);
  /* Then, setup the pins as the real alternate-function SPIs. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* GPIOF Configuration: SPI5 MOSI on PF11. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

  /* First, setup direct GPIO with line low. */
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOF, GPIO_Pin_11);
  /* Then, setup the pin as the real alternate-function SPI. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOF, &GPIO_InitStructure);

  /* GPIOG Configuration: SPI6 CLK on PG13, MISO on PG12, MOSI on PG14. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

  /* First, setup direct GPIO with lines low. */
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOG, GPIO_Pin_13|GPIO_Pin_14);
  /* Then, setup the pins as the real alternate-function SPIs. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOG, &GPIO_InitStructure);

  /* GPIOH Configuration: SPI5 CLK on PH6, MISO on PH7. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOH, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

  /* First, setup direct GPIO with lines low. */
  GPIO_Init(GPIOH, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOH, GPIO_Pin_6);
  /* Then, setup the pins as the real alternate-function SPIs. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOH, &GPIO_InitStructure);

  /* GPIOA Configuration: LAT1 on PA4. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* Latch is active high; initialise it low. */
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);

  /* GPIOD Configuration: LAT5 on PD10, LAT2 on PD11. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  /* Latch is active high; initialise it low. */
  GPIO_ResetBits(GPIOD, GPIO_Pin_10|GPIO_Pin_11);

  /* GPIOE Configuration: LAT4 on PE15. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  /* Latch is active high; initialise it low. */
  GPIO_ResetBits(GPIOE, GPIO_Pin_15);

  /* GPIOG Configuration: LAT3 on PG15. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  /* Latch is active high; initialise it low. */
  GPIO_ResetBits(GPIOG, GPIO_Pin_15);

  /* GPIOH Configuration: LAT6 on PH3. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOH, &GPIO_InitStructure);
  /* Latch is active high; initialise it low. */
  GPIO_ResetBits(GPIOH, GPIO_Pin_3);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_SPI4);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_SPI4);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_SPI4);
  GPIO_PinAFConfig(GPIOH, GPIO_PinSource6, GPIO_AF_SPI5);
  GPIO_PinAFConfig(GPIOH, GPIO_PinSource7, GPIO_AF_SPI5);
  GPIO_PinAFConfig(GPIOF, GPIO_PinSource11, GPIO_AF_SPI5);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource13, GPIO_AF_SPI6);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource12, GPIO_AF_SPI6);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_SPI6);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  /* SCLK is idle low, both setup and sample on rising edge. */
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  /* SPI1/4/5/6 is on the 90 MHz APB2, so prescalar 4 is 22.5 MHz. */
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_Init(SPI1, &SPI_InitStructure);
  SPI_Init(SPI4, &SPI_InitStructure);
  SPI_Init(SPI5, &SPI_InitStructure);
  SPI_Init(SPI6, &SPI_InitStructure);
  SPI_Cmd(SPI1, ENABLE);
  SPI_Cmd(SPI4, ENABLE);
  SPI_Cmd(SPI5, ENABLE);
  SPI_Cmd(SPI6, ENABLE);
  /* SPI2/3 is on the 45 MHz APB1, so prescalar 2 is 22.5 MHz. */
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  SPI_Init(SPI2, &SPI_InitStructure);
  SPI_Init(SPI3, &SPI_InitStructure);
  SPI_Cmd(SPI2, ENABLE);
  SPI_Cmd(SPI3, ENABLE);

  /*
    Setup DMA for SPI Tx.
    SPI1 uses DMA2 stream 3 ch 3.
    SPI2 uses DMA1 stream 4 ch 0.
    SPI3 uses DMA1 stream 5 ch 0.
    SPI4 uses DMA2 stream 1 ch 4.
    SPI5 uses DMA2 stream 4 ch 2.
    SPI6 uses DMA2 stream 5 ch 1.
  */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  DMA_DeInit(DMA2_Stream1);
  DMA_DeInit(DMA2_Stream3);
  DMA_DeInit(DMA2_Stream4);
  DMA_DeInit(DMA2_Stream5);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  DMA_DeInit(DMA1_Stream4);
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
  /* Configure SPI2 TX DMA */
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI2->DR));
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  DMA_Init(DMA1_Stream4, &DMA_InitStructure);
  /* Configure SPI3 TX DMA */
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI3->DR));
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);
  /* Configure SPI4 TX DMA */
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI4->DR));
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);
  /* Configure SPI5 TX DMA */
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI5->DR));
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  DMA_Init(DMA2_Stream4, &DMA_InitStructure);
  /* Configure SPI6 TX DMA */
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI6->DR));
  DMA_InitStructure.DMA_Channel = DMA_Channel_1;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  DMA_Init(DMA2_Stream5, &DMA_InitStructure);
}


static void
tlc_latch(GPIO_TypeDef * latch_gpio, uint16_t latch_pin)
{
  delay(1);
  GPIO_SetBits(latch_gpio, latch_pin);
  delay(1);
  GPIO_ResetBits(latch_gpio, latch_pin);
  delay(1);
}


static void
dma_to_tlc(uint8_t *outbuf, uint32_t len,
           SPI_TypeDef *spi_dev, DMA_Stream_TypeDef *dma_stream,
           uint32_t dma_flag)
{
  dma_stream->M0AR = (uint32_t)outbuf;
  dma_stream->NDTR = len;

  DMA_Cmd(dma_stream, ENABLE);
  SPI_I2S_DMACmd(spi_dev, SPI_I2S_DMAReq_Tx, ENABLE);
  while (DMA_GetFlagStatus(dma_stream, dma_flag) == RESET)
    ;
  DMA_ClearFlag(dma_stream, dma_flag);
  DMA_Cmd(dma_stream, DISABLE);
  SPI_I2S_DMACmd(spi_dev, SPI_I2S_DMAReq_Tx, DISABLE);

  while (!(spi_dev->SR & SPI_I2S_FLAG_TXE))
    ;
  while (spi_dev->SR & SPI_I2S_FLAG_BSY)
    ;

  /*
    Clear out any pending not-read data.
    (Overflow flag is cleared implicitly by reading the status register.)
  */
  while (spi_dev->SR & SPI_I2S_FLAG_RXNE)
    (void)SPI_I2S_ReceiveData(spi_dev);
}

__attribute__((unused))
static void
write_to_tlc(uint8_t *outbuf, uint32_t len,
             SPI_TypeDef *spi_dev, DMA_Stream_TypeDef *dma_stream,
             uint32_t dma_flag)
{
  /* Clear out any pending not-read data. */
  while (spi_dev->SR & SPI_I2S_FLAG_RXNE)
    (void)SPI_I2S_ReceiveData(spi_dev);

  while (len > 0)
  {
    while (!(spi_dev->SR & SPI_I2S_FLAG_TXE))
      ;
    SPI_I2S_SendData(spi_dev, *outbuf);
    while (!(spi_dev->SR & SPI_I2S_FLAG_RXNE))
      ;
    (void)SPI_I2S_ReceiveData(spi_dev);
    ++outbuf;
    --len;
  }
}


static void
read_from_tlc(SPI_TypeDef *spi_dev, uint8_t *buf, uint32_t len)
{
  /* Clear out any pending not-read data. */
  while (spi_dev->SR & SPI_I2S_FLAG_RXNE)
    (void)SPI_I2S_ReceiveData(spi_dev);

  while (len > 0)
  {
    while (!(spi_dev->SR & SPI_I2S_FLAG_TXE))
      ;
    SPI_I2S_SendData(spi_dev, 0);
    while (!(spi_dev->SR & SPI_I2S_FLAG_RXNE))
      ;
    *buf = SPI_I2S_ReceiveData(spi_dev);
    ++buf;
    --len;
  }
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
shift_buf_6_bits(uint8_t *buf, uint32_t len)
{
  uint32_t i;
  uint8_t old;

  old = 0;
  for (i = 0; i < len; ++i)
  {
    uint8_t val = buf[i];
    buf[i] = (old << 2) | (val >> 6);
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
  uint32_t i, j;
  float max_dist = led_distance_to_center_xy(6,3);

  memset(buf, 0, 193);
  for (j = 0; j < 2; ++j)
  {
    for (i = 0; i < 48; ++i)
    {
      uint32_t led = i/3;
      uint32_t dc_adj;
      /* ToDo: This needs to use an updated table, and take into account the 'j' index. */
      float dist = led_distance_to_center_tlc(tlc_idx, led);
      float dc_fact = dist / max_dist;
      /* Minimum brightness @ DC=0 is 26.2%. */
      if (dc_fact < 0.262f)
        dc_fact = 0.262f;
      dc_adj = (uint32_t)(0.5f + 127.0f * (dc_fact - 0.262f)/(1.0f - 0.262f));
      add_bits(buf, 193, dc_adj, 7, &pos);
    }
    for (i = 0; i < 3; ++i)
      add_bits(buf, 193, mc_val, 3, &pos);
    for (i = 0; i < 3; ++i)
      add_bits(buf, 193, bc_val, 7, &pos);
    /*
      Control bits:
        366 DSPRPT  0   Disable auto repeat
        367 TMGRST  1   display timing reset mode enabled (reset GS at LAT)
        368 RFRESH  0   auto data refresh disabled
        369 ESPWM   1   Enable enhanced spectrum PWM
        370 LSDVLT  0   LSD voltage is VCC * 0.7
    */
    add_bits(buf, 193, 0x0a, 5, &pos);
    /* Need 0x1 0x96 at start of buffer to latch control register. */
    pos = 760 + 769*j;
    add_bits(buf, 193, 0x196, 9, &pos);
  }
}


static void
fill_tlc5955_gs_latch(uint8_t *buf, uint32_t max_gs)
{
  uint32_t i, j, k;
  uint32_t idx;

  memset(buf, 0, 193);
  idx = 0;
  for (k = 0; k < 2; ++k)
  {
    for (i = 0; i < 16; ++i)
    {
      for (j = 0; j < 3; ++j)
      {
        uint32_t gsval = ( (i & (1<<j)) ? max_gs/(2-(i/8)) : 0);
        add_bits(buf, 193, gsval, 16, &idx);
      }
    }
    /* Add the "0" bit that selects the GS register. */
    add_bits(buf, 193, 0, 1, &idx);
  }
}


static void
setup_tlc5955(uint32_t tlc_idx, SPI_TypeDef *spi_dev,
              DMA_Stream_TypeDef *dma_stream, uint32_t dma_flag,
              GPIO_TypeDef * latch_gpio, uint16_t latch_pin)
{
  uint8_t databuf[193], inbuf[193];

  /*
    MC=4 is 19.1 mA max.
    BC=0 is 10% -> 1.91 mA.
    BC=127 is 100% -> 19.1 mA.
    Should be safe even for USB usage, or can use MC=2 BC=0 for 1.12 mA.
  */
  fill_tlc5955_control_latch(databuf, tlc_idx, led_intensity, 4);
  serial_puts("Sending control register data to TLC5955:\r\n");
  serial_dump_buf(databuf, sizeof(databuf));
  dma_to_tlc(databuf, sizeof(databuf), spi_dev, dma_stream, dma_flag);
  tlc_latch(latch_gpio, latch_pin);

  read_from_tlc(spi_dev, inbuf, sizeof(inbuf));
  tlc_latch(latch_gpio, latch_pin);
  /* Since we shift out 7 bits too many, we need to adjust the result read. */
  shift_buf_6_bits(inbuf, 193);
  if (0 != memcmp(databuf, inbuf, sizeof(databuf)))
  {
    serial_dump_buf(inbuf, sizeof(inbuf));
    serial_puts(" ouch, read control data does not match!\r\n");
    for (;;)
      ;
  }
  else
    serial_puts("Control data verified ok, rewrite to confirm config\r\n");
  dma_to_tlc(databuf, sizeof(databuf), spi_dev, dma_stream, dma_flag);
  tlc_latch(latch_gpio, latch_pin);
  serial_puts("Now load some GS data ...\r\n");
  fill_tlc5955_gs_latch(databuf, 4095);
  serial_dump_buf(databuf, sizeof(databuf));
  dma_to_tlc(databuf, sizeof(databuf), spi_dev, dma_stream, dma_flag);
  tlc_latch(latch_gpio, latch_pin);
#if EXTRA_DEBUG
  /* Read out and dump the status information, for debugging. */
  read_from_tlc(spi_dev, inbuf, sizeof(inbuf));
  serial_puts("SID data from TLCs:\r\n");
  shift_buf_6_bits(inbuf, sizeof(inbuf));
  serial_dump_buf(inbuf, sizeof(inbuf));
#endif
}


void
setup_spi(void)
{
  setup_tlc_spi_dma();

  serial_puts("Setting up TLC U8/U9 on SPI1...\r\n");
  setup_tlc5955(0, SPI1, DMA2_Stream3, DMA_FLAG_TCIF3, GPIOA, GPIO_Pin_4);
  serial_puts("Setting up TLC U12/U13 on SPI2...\r\n");
  setup_tlc5955(1, SPI2, DMA1_Stream4, DMA_FLAG_TCIF4, GPIOD, GPIO_Pin_11);
  serial_puts("Setting up TLC U6/U7 on SPI3...\r\n");
  setup_tlc5955(2, SPI3, DMA1_Stream5, DMA_FLAG_TCIF5, GPIOG, GPIO_Pin_15);
  /* ToDo: Solder pad on U10/U11 (SPI4) is ruined (?) */
  serial_puts("Setting up TLC U10/U11 on SPI4...\r\n");
  setup_tlc5955(0 /* ToDo */, SPI4, DMA2_Stream1, DMA_FLAG_TCIF1, GPIOE, GPIO_Pin_15);
  serial_puts("Setting up TLC U14/U15 on SPI5...\r\n");
  setup_tlc5955(1 /* ToDo */, SPI5, DMA2_Stream4, DMA_FLAG_TCIF4, GPIOD, GPIO_Pin_10);
  serial_puts("Setting up TLC U4/U5 on SPI6...\r\n");
  setup_tlc5955(2 /* ToDo */, SPI6, DMA2_Stream5, DMA_FLAG_TCIF5, GPIOH, GPIO_Pin_3);
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
start_dma_scanplanes(uint32_t *p1, uint32_t *p2, uint32_t *p3,
                     uint32_t *p4, uint32_t *p5, uint32_t *p6)
{
  static const uint32_t len = 2*48*2+2;/* 2 TLCs each 48 outputs + extra word */

  DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
  DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
  DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
  DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1);
  DMA_ClearFlag(DMA2_Stream4, DMA_FLAG_TCIF4);
  DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);

  DMA2_Stream3->M0AR = (uint32_t)p1;
  DMA2_Stream3->NDTR = len;
  DMA1_Stream4->M0AR = (uint32_t)p2;
  DMA1_Stream4->NDTR = len;
  DMA1_Stream5->M0AR = (uint32_t)p3;
  DMA1_Stream5->NDTR = len;
  DMA2_Stream1->M0AR = (uint32_t)p4;
  DMA2_Stream1->NDTR = len;
  DMA2_Stream4->M0AR = (uint32_t)p5;
  DMA2_Stream4->NDTR = len;
  DMA2_Stream5->M0AR = (uint32_t)p6;
  DMA2_Stream5->NDTR = len;

  DMA_Cmd(DMA2_Stream3, ENABLE);
  DMA_Cmd(DMA1_Stream4, ENABLE);
  DMA_Cmd(DMA1_Stream5, ENABLE);
  DMA_Cmd(DMA2_Stream1, ENABLE);
  DMA_Cmd(DMA2_Stream4, ENABLE);
  DMA_Cmd(DMA2_Stream5, ENABLE);
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(SPI4, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(SPI5, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(SPI6, SPI_I2S_DMAReq_Tx, ENABLE);
}


void
latch_scanplanes(void)
{
  delay(1);
  GPIO_SetBits(GPIOA, GPIO_Pin_4);
  GPIO_SetBits(GPIOD, GPIO_Pin_11);
  GPIO_SetBits(GPIOG, GPIO_Pin_15);
  GPIO_SetBits(GPIOE, GPIO_Pin_15);
  GPIO_SetBits(GPIOD, GPIO_Pin_10);
  GPIO_SetBits(GPIOH, GPIO_Pin_3);
  delay(1);
  GPIO_ResetBits(GPIOA, GPIO_Pin_4);
  GPIO_ResetBits(GPIOD, GPIO_Pin_11);
  GPIO_ResetBits(GPIOG, GPIO_Pin_15);
  GPIO_ResetBits(GPIOE, GPIO_Pin_15);
  GPIO_ResetBits(GPIOD, GPIO_Pin_10);
  GPIO_ResetBits(GPIOH, GPIO_Pin_3);
}


uint32_t
is_tlc_dma_done(void)
{
  if (DMA_GetFlagStatus(DMA2_Stream3, DMA_FLAG_TCIF3) == RESET ||
      DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4) == RESET ||
      DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG_TCIF5) == RESET ||
      DMA_GetFlagStatus(DMA2_Stream1, DMA_FLAG_TCIF1) == RESET ||
      DMA_GetFlagStatus(DMA2_Stream4, DMA_FLAG_TCIF4) == RESET ||
      DMA_GetFlagStatus(DMA2_Stream5, DMA_FLAG_TCIF5) == RESET)
    return 0;
  if (!(SPI1->SR & SPI_I2S_FLAG_TXE) ||
      !(SPI2->SR & SPI_I2S_FLAG_TXE) ||
      !(SPI3->SR & SPI_I2S_FLAG_TXE) ||
      !(SPI4->SR & SPI_I2S_FLAG_TXE) ||
      !(SPI5->SR & SPI_I2S_FLAG_TXE) ||
      !(SPI6->SR & SPI_I2S_FLAG_TXE))
    return 0;
  if ((SPI1->SR & SPI_I2S_FLAG_BSY) ||
      (SPI2->SR & SPI_I2S_FLAG_BSY) ||
      (SPI3->SR & SPI_I2S_FLAG_BSY) ||
      (SPI4->SR & SPI_I2S_FLAG_BSY) ||
      (SPI5->SR & SPI_I2S_FLAG_BSY) ||
      (SPI6->SR & SPI_I2S_FLAG_BSY))
    return 0;
  return 1;
}
