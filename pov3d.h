#include <stm32f4_discovery.h>

#define LILLE_VIDUNDER

#ifdef STM32F4_DISCOVERY
#define LED_PERIPH RCC_AHB1Periph_GPIOD
#define LED_GPIO GPIOD
#define LED_PIN GPIO_Pin_12
#endif

#ifdef LILLE_VIDUNDER
#define LED_PERIPH RCC_AHB1Periph_GPIOG
#define LED_GPIO GPIOG
#define LED_PIN GPIO_Pin_15
#endif

#ifdef PCB_POV3D
#define LED_PERIPH RCC_AHB1Periph_GPIOC
#define LED_GPIO GPIOC
#define LED_PIN GPIO_Pin_13
#endif


/* misc.c */
extern void delay(__IO uint32_t nCount);

/* led.c */
void setup_led(void);
extern void led_on(void);
extern void led_off(void);
