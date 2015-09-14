/* POV3D. */

#include "ledtorus.h"


static uint8_t led_intensity = 100;

static void
led_decrease_intensity(void)
{
  /* ToDo */
}


static void
led_increase_intensity(void)
{
  /* ToDo */
}


int
main(void)
{
  uint32_t led_state;
  uint32_t old_frame_counter;
  static const uint32_t NUM_ANIMS = 3;
  uint32_t cur_anim = 0;

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
  serial_puts("Initialising nRF24L01+ wireless communications...\r\n");
  setup_nrf24l01p();
  serial_puts("Setting up SD card...\r\n");
  setup_sd_sdio();
  serial_puts("Setting up Hall sensor...\r\n");
  setup_hall();
  serial_puts("Setup done, starting loop...\r\n");

  led_state = 0;
  test_img1();
  old_frame_counter = 42;
  for (;;)
  {
    uint32_t new_frame_counter;
    uint32_t key;

    do
    {
      new_frame_counter = get_frame_counter();
    } while (new_frame_counter == old_frame_counter);
    old_frame_counter = new_frame_counter;

    if (key_state & (1<<5))
    {
      /* Manual mode. Animation selected by keys 0 and 1. */
    }
    else
    {
      /* Automatic mode, animation switches after playing for some time. */
      uint32_t x = (led_state % 2048);
      if (x < 512)
        cur_anim = 0;
      else if (x < 1024)
        cur_anim = 1;
      else
        cur_anim = 2;
    }
    switch(cur_anim)
    {
    case 0:
      an_supply_voltage(render_framebuf(), led_state, NULL);
      break;
    case 1:
      an_ghost(render_framebuf(), led_state, NULL);
      break;
    case 2:
      an_sdcard(render_framebuf(), led_state, NULL);
      break;
    }

    while ((key = get_key_event()) != KEY_NOEVENT)
    {
      serial_puts("K: 0x");
      print_uint32_hex(key);
      serial_puts(" 0x");
      serial_output_hexbyte(key_state);
      serial_puts("\r\n");

      if (key == (0 | KEY_EVENT_DOWN))
        cur_anim = (cur_anim + (NUM_ANIMS-1)) % NUM_ANIMS;
      else if (key == (1 | KEY_EVENT_DOWN))
        cur_anim = (cur_anim + 1) % NUM_ANIMS;
      else if (key == (2 | KEY_EVENT_DOWN))
        led_decrease_intensity();
      else if (key == (3 | KEY_EVENT_DOWN))
        led_increase_intensity();
    }

    if (!(led_state % 25))
    {
      float val;

      serial_puts("V: ");
      val = voltage_read();
      println_float(val, 1, 3);
    }

    ++led_state;
  }
}
