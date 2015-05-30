/* POV3D. */

#include "pov3d.h"

/* This is apparently needed for libc/libm (eg. powf()). */
int __errno;


int
main(void)
{
  uint32_t led_state;
  uint32_t led_count;

  setup_led();
  setup_serial();

  {
    /* Quick hack to pulse BLANK on PB0. */
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_0);
  }
  serial_puts("\r\n\r\nPOV3D Copyright 2015 Kristian Nielsen\r\n");
  serial_puts("Setting up TLCs...\r\n");
  setup_spi();
  serial_puts("Configuring ADC...\r\n");
  config_adc();
  serial_puts("Starting GSCLKs...\r\n");
  setup_gsclks();
  serial_puts("Setup done, starting loop...\r\n");
  led_state = 0;
  led_count = 0;
  for (;;)
  {
    GPIO_SetBits(GPIOB, GPIO_Pin_0);
    delay(2);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    delay(4096*GSCLK_PERIOD*2/3);

    ++led_count;
    if (led_count >= 30000000/(4096*GSCLK_PERIOD))
    {
      if (led_state)
      {
        uint32_t val;

        led_on();
        val = adc_read();
        println_uint32(val);
      }
      else
      {
        led_off();
      }
      led_state = led_state ^ 1;
      serial_putchar('.');
      led_count = 0;
    }
  }
}
