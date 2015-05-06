#include <stm32f4_discovery.h>

/* misc.c */
extern void delay(__IO uint32_t nCount);

/* led.c */
#define LED_PERIPH RCC_AHB1Periph_GPIOD
#define LED_GPIO GPIOD
#define LED_PIN GPIO_Pin_12

void setup_led(void);
extern void led_on(void);
extern void led_off(void);
