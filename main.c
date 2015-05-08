/* POV3D. */

#include "pov3d.h"

/* This is apparently needed for libc/libm (eg. powf()). */
int __errno;


int
main(void)
{
  setup_led();
  setup_serial();

  serial_puts("\r\n\r\nPOV3D Copyright 2015 Kristian Nielsen\r\n");
  serial_puts("Setting up TLCs...\r\n");
  setup_spi();
  serial_puts("Setup done, starting loop...\r\n");
  for (;;)
  {
    led_on();
    delay(10000000);
    led_off();
    delay(10000000);
    serial_putchar('.');
  }
}
