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


/*
  In the LED-torus prototype, we have 7 (horizontal) x 8 (vertical) RGB LEDs.
  But the corners are cut, so that these LEDs are missing, counting the X
  direction from the center and out, and the Y direction from the bottom up:

    (0,0) (0,1) (0,6) (0,7) (1,0) (1,7) (6,0) (6,7)

         * * * *
       * * * * * *
     * * * * * * *
     * * * * * * *
     * * * * * * *
     * * * * * * *
       * * * * * *
         * * * *
*/
#define LEDS_X 7
#define LEDS_Y 8
#define LEDS_TANG 335


typedef uint8_t frame_t[LEDS_Y*LEDS_X*LEDS_TANG][3];


/* Based on TIM5, running at 84 MHz. */
#define GSCLK_PERIOD 4


#define F_PI 3.141592654f


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
extern void print_uint32_hex(uint32_t val);
extern void float_to_str(char *buf, float f,
                         uint32_t dig_before, uint32_t dig_after);
extern void println_float(float f, uint32_t dig_before, uint32_t dig_after);
extern void serial_dump_buf(uint8_t *buf, uint32_t len);

/* spi.c */
extern void setup_spi(void);
extern void start_dma_scanplanes(uint32_t *p1, uint32_t *p2, uint32_t *p3);
extern void latch_scanplanes(void);
extern uint32_t is_tlc_dma_done(void);

/* timers.c */
extern void setup_timers(void);
extern uint32_t get_frame_counter(void);
extern void tlc_show_time_stat(void);
static inline uint32_t
get_time(void)
{
  return SysTick->VAL;
}
static inline uint32_t
calc_time_from_val(uint32_t start, uint32_t stop)
{
  return (start - stop) & 0xffffff;
}


static inline uint32_t
calc_time(uint32_t start)
{
  uint32_t stop = get_time();
  return calc_time_from_val(start, stop);
}


/* adc.c */
extern void config_adc(void);
extern uint32_t adc_read(void);
extern float voltage_read(void);

/* tlc.c */
extern void make_scan_planes(uint32_t angle, uint32_t *b1, uint32_t *b2,
                             uint32_t *b3);
extern void init_tlc(void);
extern float led_distance_to_center_xy(uint32_t x, uint32_t y);
extern float led_distance_to_center_tlc(uint32_t tlc, uint32_t output);
extern uint8_t (*render_framebuf(void))[LEDS_Y*LEDS_X*LEDS_TANG][3];
extern uint8_t (*display_framebuf(void))[LEDS_Y*LEDS_X*LEDS_TANG][3];
extern void flip_framebuf(void);

/* gfx.c */
extern void test_img1(void);
extern void an_ghost(frame_t *f, uint32_t c, void *st);
extern void an_supply_voltage(frame_t *f, uint32_t c, void *st);

/* font_tonc.c */
extern const uint8_t tonc_font[8*96];
