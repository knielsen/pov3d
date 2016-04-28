/* POV3D. */

#include "ledtorus.h"


int
main(void)
{
  uint32_t anim_state;
  uint32_t old_frame_counter;
  uint32_t cur_anim;
  uint32_t anim_running;

  setup_led();
  setup_serial();

  serial_puts("\r\n\r\nPOV3D Copyright 2015 Kristian Nielsen\r\n");
  serial_puts("Setting up TLCs...\r\n");
  setup_spi();
//  init_tlc();
  serial_puts("Configuring ADC...\r\n");
  config_adc();
//  serial_puts("Starting timers...\r\n");
//  setup_timers();
//  serial_puts("Initialising nRF24L01+ wireless communications...\r\n");
//  setup_nrf24l01p();
//  serial_puts("Setting up SD card...\r\n");
//  setup_sd_sdio();
//  serial_puts("Setting up Hall sensor...\r\n");
//  setup_hall();
  serial_puts("Setup done, starting loop...\r\n");

  for (;;)
  {
    float val;

    led_on();
    delay(MCU_HZ/3/4);
    led_off();
    delay(MCU_HZ/3/4);
    val = voltage_read_vrefint_adjust();
    println_float(val, 1, 3);
  }

  anim_state = 0;
  test_img1();
  old_frame_counter = 42;
  anim_running = 0;
  cur_anim = anim_table_length - 1;
  for (;;)
  {
    uint32_t new_frame_counter;
    uint32_t key;

    do
    {
      new_frame_counter = get_frame_counter();
    } while (new_frame_counter == old_frame_counter);
    old_frame_counter = new_frame_counter;

#ifdef SERIAL_DBG
    if (!(anim_state % 100))
    {
      float val;

      serial_puts("V: ");
      val = voltage_read();
      println_float(val, 1, 3);
    }
#endif

    while (!anim_running)
    {
      ++cur_anim;
      if (cur_anim >= anim_table_length)
        cur_anim = 0;
      anim_state = 0;
      anim_running = !anim_init(cur_anim);
    }
    anim_running = !anim_nextframe(cur_anim, render_framebuf(), anim_state);

    ++anim_state;
    if (key_state[0] & (1<<5))
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
#ifdef SERIAL_DBG
      serial_puts("K: 0x");
      print_uint32_hex(key);
      serial_puts(" 0x");
      serial_output_hexbyte(key_state[0]);
      serial_output_hexbyte(key_state[1]);
      serial_output_hexbyte(key_state[2]);
      serial_puts("\r\n");
#endif

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
