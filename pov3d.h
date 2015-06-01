#include <math.h>
#include <string.h>
#include <stm32f4_discovery.h>

#define PCB_POV3D
//#define LILLE_VIDUNDER

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


/* Based on TIM5, running at 84 MHz. */
#define GSCLK_PERIOD 4


/* misc.c */
extern void delay(__IO uint32_t nCount);

/* led.c */
extern void setup_led(void);
extern void led_on(void);
extern void led_off(void);

/* dbg.c */
extern void setup_serial(void);
extern void serial_putchar(uint32_t c);
extern void serial_puts(const char *s);
extern void serial_output_hexbyte(uint8_t byte);
extern void println_uint32(uint32_t val);
extern void println_int32(int32_t val);
extern void println_float(float f, uint32_t dig_before, uint32_t dig_after);
extern void serial_dump_buf(uint8_t *buf, uint32_t len);

/* spi.c */
extern void setup_spi(void);

/* timers.c */
extern void setup_gsclks(void);

/* adc.c */
extern void config_adc(void);
extern uint32_t adc_read(void);
extern float voltage_read(void);
