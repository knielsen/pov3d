#include "ledtorus.h"


void
setup_led(void)
{
  rcc_periph_clock_enable(LED_PERIPH);
  gpio_mode_setup(LED_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
  gpio_set_output_options(LED_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, LED_PIN);
}


void
led_on(void)
{
  gpio_set(LED_GPIO, LED_PIN);
}


void
led_off(void)
{
  gpio_clear(LED_GPIO, LED_PIN);
}
