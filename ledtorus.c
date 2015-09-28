/* POV3D. */

#include "ledtorus.h"


static uint8_t led_intensity = 50;

static void
led_decrease_intensity(void)
{
  if (led_intensity > 0)
  {
    uint8_t intensity_inc = (led_intensity > 15 ? 10 : 2);
    if (led_intensity > intensity_inc)
      led_intensity -= intensity_inc;
    else
      led_intensity = 0;
    new_intensity(led_intensity);
  }
}


static void
led_increase_intensity(void)
{
  if (led_intensity < 127)
  {
    uint8_t intensity_dec = (led_intensity > 24 ? 10 : 2);
    if ((127 - led_intensity) > intensity_dec)
      led_intensity += intensity_dec;
    else
      led_intensity = 127;
    new_intensity(led_intensity);
  }
}


int
main(void)
{
  uint32_t anim_state;
  uint32_t old_frame_counter;
  uint32_t cur_anim;
  uint32_t anim_running;
  void *cur_anim_data;

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

  anim_state = 0;
  test_img1();
  old_frame_counter = 42;
  anim_running = 0;
  cur_anim = anim_table_length - 1;
  cur_anim_data = NULL;
  for (;;)
  {
    uint32_t new_frame_counter;
    uint32_t key;

    do
    {
      new_frame_counter = get_frame_counter();
    } while (new_frame_counter == old_frame_counter);
    old_frame_counter = new_frame_counter;

    if (!(anim_state % 100))
    {
      float val;

      serial_puts("V: ");
      val = voltage_read();
      println_float(val, 1, 3);
    }

    while (!anim_running)
    {
      ++cur_anim;
      if (cur_anim >= anim_table_length)
        cur_anim = 0;
      anim_state = 0;
      if (anim_table[cur_anim].init)
        anim_running = !anim_table[cur_anim].init(&anim_table[cur_anim],
                                                  &cur_anim_data);
      else
        anim_running = 1;
    }
    anim_running = !anim_table[cur_anim].nextframe(render_framebuf(),
                                                   anim_state, cur_anim_data);

    ++anim_state;
    if (key_state & (1<<5))
    {
      /* Manual mode. Animation selected by keys 0 and 1. */
    }
    else
    {
      /* Automatic mode, animation switches after playing for some time. */
      if (anim_table[cur_anim].duration > 0 &&
          anim_state >= anim_table[cur_anim].duration)
        anim_running = 0;
    }

    while ((key = get_key_event()) != KEY_NOEVENT)
    {
      serial_puts("K: 0x");
      print_uint32_hex(key);
      serial_puts(" 0x");
      serial_output_hexbyte(key_state);
      serial_puts("\r\n");

      if (key == (0 | KEY_EVENT_DOWN))
      {
        cur_anim = (cur_anim + anim_table_length - 2) % anim_table_length;
        anim_running = 0;
      }
      else if (key == (1 | KEY_EVENT_DOWN))
      {
        anim_running = 0;
      }
      else if (key == (2 | KEY_EVENT_DOWN))
        led_decrease_intensity();
      else if (key == (3 | KEY_EVENT_DOWN))
        led_increase_intensity();
    }
  }
}
