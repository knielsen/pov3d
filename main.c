/* POV3D. */

#include "pov3d.h"


int
main(void)
{
  setup_led();

  led_on();
  for (;;)
  {
    led_on();
    delay(10000000);
    led_off();
    delay(10000000);
  }
}
