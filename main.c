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
    delay(4096*GSCLK_PERIOD*2/3);

    ++led_count;
    if (led_count >= 30000000/(4096*GSCLK_PERIOD))
    {
      if (led_state)
      {
        float val;

        led_on();
        val = voltage_read();
        println_float(val, 1, 3);
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
