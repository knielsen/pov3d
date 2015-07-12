/* POV3D. */

#include "ledtorus.h"


int
main(void)
{
  uint32_t led_state;
  uint32_t old_frame_counter;

  setup_led();
  setup_serial();

  serial_puts("\r\n\r\nPOV3D Copyright 2015 Kristian Nielsen\r\n");
  serial_puts("Setting up TLCs...\r\n");
  setup_spi();
  init_tlc();
  serial_puts("Configuring ADC...\r\n");
  config_adc();
  serial_puts("Starting timers...\r\n");
  setup_timers();
  serial_puts("Setup done, starting loop...\r\n");

  led_state = 0;
  test_img1();
  old_frame_counter = 42;
  for (;;)
  {
    uint32_t new_frame_counter;
    do
    {
      new_frame_counter = get_frame_counter();
    } while (new_frame_counter == old_frame_counter);
    old_frame_counter = new_frame_counter;

    if ((led_state % 512) < 256)
      an_supply_voltage(render_framebuf(), led_state, NULL);
    else
      an_ghost(render_framebuf(), led_state, NULL);

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

    ++led_state;
    serial_putchar('.');
  }
}
