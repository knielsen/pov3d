#include <math.h>
#include <string.h>
#include <stm32f4xx.h>

#define PCB_POV3D
//#define LILLE_VIDUNDER

//#define SERIAL_DBG
//#define DEBUG_SPEED_STABILITY 1

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

#ifdef PCB_POV3D_2
#define LED_PERIPH RCC_AHB1Periph_GPIOE
#define LED_GPIO GPIOE
#define LED_PIN GPIO_Pin_2
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
#define LEDS_TANG 205
#define FRAMERATE 25

/*
  Radio channel to use for nRF24L01+. Must match peer side.
  Range is 0-125 for 2400-2525 MHz.
*/
#ifndef NRF_CHANNEL
#define NRF_CHANNEL 81
#endif


#define FRAMEBUF_SIZE (LEDS_Y*LEDS_X*LEDS_TANG*3)
#define DMA_FRAMEBUF_SIZE ((FRAMEBUF_SIZE+511)/512*128)
typedef uint8_t frame_t[LEDS_Y*LEDS_X*LEDS_TANG][3];


/* Based on TIM5, running at 84 MHz. */
#define GSCLK_PERIOD 4


#define F_PI 3.141592654f


/*
  Note that changing frequence requires more code changes than just changing
  this constant.
*/
#define MCU_HZ 168000000


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
extern char *float_to_str(char *buf, float f,
                         uint32_t dig_before, uint32_t dig_after);
extern void println_float(float f, uint32_t dig_before, uint32_t dig_after);
extern void serial_dump_buf(uint8_t *buf, uint32_t len);

/* spi.c */
extern void setup_spi(void);
extern void start_dma_scanplanes(uint32_t *p1, uint32_t *p2, uint32_t *p3);
extern void latch_scanplanes(void);
extern uint32_t is_tlc_dma_done(void);
extern void fill_tlc5955_control_latch(uint8_t *buf, uint32_t tlc_idx,
                                       uint32_t bc_val, uint32_t mc_val);

/* timers.c */
extern void setup_timers(void);
extern uint32_t get_frame_counter(void);
extern void tlc_show_time_stat(void);
extern void new_intensity(uint8_t intensity);
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
extern float voltage_read_vrefint_adjust(void);

/* tlc.c */
extern uint8_t led_intensity;
extern void make_scan_planes(uint32_t angle, uint32_t *b1, uint32_t *b2,
                             uint32_t *b3);
extern void init_tlc(void);
extern float led_distance_to_center_xy(uint32_t x, uint32_t y);
extern float led_distance_to_center_tlc(uint32_t tlc, uint32_t output);
extern uint8_t (*render_framebuf(void))[LEDS_Y*LEDS_X*LEDS_TANG][3];
extern uint8_t (*display_framebuf(void))[LEDS_Y*LEDS_X*LEDS_TANG][3];
extern void flip_framebuf(void);
extern void led_decrease_intensity(void);
extern void led_increase_intensity(void);

/* gfx.c */
union anim_data;
extern const struct ledtorus_anim {
  const char *name;
  const char *description;
  uint32_t duration;
  void *init_data;
  uint32_t (*init)(const struct ledtorus_anim *self, union anim_data *data);
  uint32_t (*nextframe)(frame_t *f, uint32_t c, union anim_data *data);
} anim_table[];
extern const uint32_t anim_table_length;
extern void test_img1(void);
extern uint32_t anim_init(uint32_t anim_idx);
extern uint32_t anim_nextframe(uint32_t anim_idx, frame_t *f, uint32_t anim_state);

/* font_tonc.c */
extern const uint8_t tonc_font[8*96];

/* nrf24l01p.c */
#define KEY_EVENT_DOWN ((uint32_t)0x80000000)
#define KEY_NOEVENT ((uint32_t)0xffffffff)
extern volatile uint8_t key_state[19];
extern uint32_t get_key_event(void);
extern void setup_nrf24l01p(void);
extern float joy_r_angle_mag(float *magnitude);
extern float joy_l_vert(void);
static inline uint32_t key_cross_state(void) {
  return key_state[2] & 0x40;
}


/* sd_sdio.c */
extern void setup_sd_sdio(void);
extern int read_sectors(uint32_t *buf, uint32_t count);
extern int open_file(const char *name);

/* hall.c */
extern volatile uint32_t prev_hall;
extern volatile uint32_t prev_hall_period;
static inline uint32_t current_hall_timer(void) { return TIM2->CNT; }
extern void setup_hall(void);
extern uint32_t check_hall(void);
extern uint32_t last_hall_period(void);
