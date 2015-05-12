#include "pov3d.h"


/*
  TLCs are on:
    PA5 (CLK) PA6 (MISO) PA7 (MOSI) PA4 (LAT)
*/


static void
setup_tlc(uint32_t for_read)
{
  /* ToDo: make the SPI a parameter so can be used for all 3. */

  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

  SPI_Cmd(SPI1, DISABLE);
  /* GPIOA Configuration: SPI1 CLK on PA5, MISO on PA6, MOSI on PA7. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
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
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = (for_read ? SPI_CPHA_2Edge : SPI_CPHA_1Edge);
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);
  SPI_Cmd(SPI1, ENABLE);
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


static void
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
  /* Send a dummy byte first, will be shifted out at the other end... */
  status_buf[0] = 0;
  send_to_tlcs(status_buf, status_buf, 1);
  send_to_tlcs(dc_buf, status_buf, sizeof(dc_buf));
  tlc_latch();
  serial_puts("Data back from sending DC:\r\n");
  serial_dump_buf(status_buf, sizeof(dc_buf));

  serial_puts("Now latch dummy GS data, to put status info in shift register.\r\n");
  memset(status_buf, 0x00, sizeof(status_buf));
  tlc_mode_gs();
  send_to_tlcs(status_buf, status_buf, sizeof(status_buf));
  tlc_latch();
  serial_puts("(Data back from GS latch:)\r\n");
  serial_dump_buf(status_buf, sizeof(status_buf));
  serial_puts("Now reading back status info from the TLCs...\r\n");
  memset(status_buf, 0x00, sizeof(status_buf));
  setup_tlc(1);
  send_to_tlcs(status_buf, status_buf, sizeof(status_buf));
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
  setup_tlc(0);
  status_buf[0] = 0;
  send_to_tlcs(status_buf, status_buf, 1);
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
  send_to_tlcs(status_buf, status_buf, sizeof(status_buf));
  tlc_latch();

  serial_puts("Dat's all .oO\r\n");
}


void
setup_spi(void)
{
  setup_tlc(0);
  setup_tcl_vprg();
  /* ToDo: Setup two remaining TLCs, and nRF. */

  setup_tlc_dc();
}
