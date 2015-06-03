/* POV3D. */

#include "pov3d.h"

/* This is apparently needed for libc/libm (eg. powf()). */
int __errno;


static void
test_scan_plane(uint32_t s)
{
  uint32_t b[25];
  uint32_t i;
  uint16_t *p = (uint16_t *)(&b[0]);

  latch_scanplanes();

  p[0] = 0;
  for (i = 1; i <= 48; ++i)
  {
    uint16_t val;
    val = ((i + s) % 3 ? 0 : 4095);
    p[i] = __REV16(val);
  }
  // ToDo check if DMA is done before starting new.
  start_dma_scanplanes(b, b, b);
}


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
      if (led_state & 1)
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
      /* After first transfer, skip subsequent if old one is not done. */
      if (led_state == 0 || is_tlc_dma_done())
        test_scan_plane(led_state);
      else
        serial_puts("Hm, skip due to DMA not completed?!?\r\n");

      ++led_state;
      serial_putchar('.');
      led_count = 0;
    }
  }
}
