/*
  Timers.

  We use TIM5 ch1-3 (on PA0-2) for GSCLKs.

  TIM5 is on the 42 MHz bus, so max timer frequency is 84 MHz.
  To get 50% duty cycle (and for GPIO to keep up?), max GSCLK is then 21 MHz.
  This is obtained with a PWM period of 4.
*/

#include "ledtorus.h"

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>


static volatile uint32_t scan_counter;

#define NOMINAL_PERIOD ((MCU_HZ/2 + (LEDS_TANG*FRAMERATE/2)) / (LEDS_TANG*FRAMERATE))
/* Set limits so that if Hall is completely off we can ignore it. */
#define MIN_PERIOD (NOMINAL_PERIOD*8/10)
#define MAX_PERIOD 64000


static void
setup_gsclks(void)
{
  rcc_periph_clock_enable(RCC_TIM5);
  rcc_periph_clock_enable(RCC_GPIOA);

  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0|GPIO1|GPIO2);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ,
                          GPIO0|GPIO1|GPIO2);
  gpio_set_af(GPIOA, GPIO_AF2, GPIO0|GPIO1|GPIO2);

  timer_set_period(TIM5, GSCLK_PERIOD-1);
  timer_set_prescaler(TIM5, 0);
  timer_set_clock_division(TIM5, 0);
  timer_direction_up(TIM5);

  timer_disable_oc_output(TIM5, TIM_OC1);
  timer_set_oc_mode(TIM5, TIM_OC1, TIM_OCM_PWM1);
  timer_set_oc_value(TIM5, TIM_OC1, GSCLK_PERIOD/2);
  timer_set_oc_polarity_high(TIM5, TIM_OC1);
  timer_enable_oc_output(TIM5, TIM_OC1);

  timer_disable_oc_output(TIM5, TIM_OC2);
  timer_set_oc_mode(TIM5, TIM_OC2, TIM_OCM_PWM1);
  timer_set_oc_value(TIM5, TIM_OC2, GSCLK_PERIOD/2);
  timer_set_oc_polarity_high(TIM5, TIM_OC2);
  timer_enable_oc_output(TIM5, TIM_OC2);

  timer_disable_oc_output(TIM5, TIM_OC3);
  timer_set_oc_mode(TIM5, TIM_OC3, TIM_OCM_PWM1);
  timer_set_oc_value(TIM5, TIM_OC3, GSCLK_PERIOD/2);
  timer_set_oc_polarity_high(TIM5, TIM_OC3);
  timer_enable_oc_output(TIM5, TIM_OC3);

  timer_enable_oc_preload(TIM5, TIM_OC1);
  timer_enable_oc_preload(TIM5, TIM_OC2);
  timer_enable_oc_preload(TIM5, TIM_OC3);
  timer_enable_preload(TIM5);
  timer_enable_counter(TIM5);
}


static void
setup_scanplane_timer(void)
{
  rcc_periph_clock_enable(RCC_TIM6);

  timer_set_period(TIM6, NOMINAL_PERIOD);
  timer_set_prescaler(TIM6, 0);
  /* No clock division or direction can be configured for basic timer TIM6. */

  timer_disable_preload(TIM6);
  timer_continuous_mode(TIM6);
  timer_update_on_overflow(TIM6);
  timer_enable_update_event(TIM6);

  scb_set_priority_grouping(SCB_AIRCR_PRIGROUP_GROUP16_NOSUB);
  nvic_set_priority(NVIC_TIM6_DAC_IRQ, 4<<4);
  nvic_enable_irq(NVIC_TIM6_DAC_IRQ);
  timer_enable_irq(TIM6, TIM_DIER_UIE);

  timer_enable_counter(TIM6);
}


static void
trigger_softint(void)
{
  /*
    Clear the pending interrupt bit before triggering the software interrupt.
    The write to SWIER only triggers an interrupt if it does not already
    have the bit set.
    Testing showed that the bit had to be cleared at least once at setup, or
    the interrupt was never triggered. Probably resetting it here again is
    not necessary, but better safe than sorry.
  */
  EXTI_PR = EXTI0;
  EXTI_SWIER = EXTI0;
}


static uint32_t scanplane_buffers[2][3][25];
static uint8_t scanbuffer_idx = 0;
static uint8_t init_counter = 0;
static uint8_t hall_sync_adj = 0;
static volatile uint32_t frame_counter = 0;
static volatile uint32_t change_intensity_flag = 0;
static uint32_t generate_scan_counter = 0;

void tim6_dac_isr(void)
{
  if (TIM6_SR & TIM_SR_UIF)
  {
    uint32_t hall_timer, c, loc_prev_hall;
    uint32_t old_fraction, new_fraction;
    uint8_t idx;
    uint8_t ic;
    static uint32_t saved_prev_hall = 0;
    static uint32_t period_int = NOMINAL_PERIOD;
    static uint32_t period_frac = 0;
    static uint32_t fractional_period = 0x7fffffff;
    static uint32_t delayed_hall = 0;

    hall_timer = current_hall_timer();
    ic = init_counter;
    if (ic >= 2)
      latch_scanplanes();

    idx = scanbuffer_idx;
    if (ic >= 2 && !is_tlc_dma_done())
    {
      serial_putchar('!');
    }
    else if (ic >= 1)
    {
      start_dma_scanplanes(scanplane_buffers[idx][0],
                           scanplane_buffers[idx][1],
                           scanplane_buffers[idx][2]);
    }

    if (ic < 2)
    {
      ++ic;
      init_counter = ic;
    }
    idx = 1 - idx;
    scanbuffer_idx = idx;

    c = scan_counter;
    loc_prev_hall = prev_hall;
    if (loc_prev_hall != saved_prev_hall)
    {
      /*
        We got the hall-sensor signal since last scanplane interrupt.
        Timers are set up to target that the hall signal comes at exactly the
        same time as the scanplane interrupt that needs to latch scanplane
        a=(LEDS_TANG-3) / generate scanplane a=(LEDS_TANG-1). Since we
        increment the scan_counter variable before, this corresponds to
        scan_counter==LEDS_TANG-2.

        However, motor speed jitter can cause the hall signal to arrive
        slightly earlier or later, and we want to adjust scanplane interrupt
        period to adjust for this jitter, in order to make the LED pattern as
        stable as possible. The hall signal may be detected in interrupt
        scan_counter==LEDS_TANG-2 or scan_counter==LEDS_TANG-1 (or other
        values if the motor speed has a lot of jitter). In either case, we
        adjust the coming scanplane interrupt periods to try and the get
        interrupt for scan_counter==1 to hit exactly 3 voxels after the
        hall signal, so we get stable LEDs from that point on.
      */
      if (c < LEDS_TANG-2)
      {
        generate_scan_counter = c+1;
        scan_counter = LEDS_TANG-1;
      }
      else if (c == LEDS_TANG-2)
      {
        generate_scan_counter = scan_counter = c+1;
      }
      else
      {
        generate_scan_counter = scan_counter = 0;
        saved_prev_hall = loc_prev_hall;
        delayed_hall = 0;
      }
      if (!hall_sync_adj)
      {
        uint32_t hall_period = prev_hall_period;
        uint32_t target_time;
        uint32_t ints_remain;
        float period;
        ints_remain = ((c == LEDS_TANG-1) ? 2 : 3);
        target_time = loc_prev_hall + 3*hall_period/LEDS_TANG - hall_timer;
        period = (float)target_time/(float)ints_remain;
        period_int = (uint32_t)period;
        if (hall_period > 0x50000000 ||
            period_int < 1500 || period_int > MAX_PERIOD)
        {
          period_int = NOMINAL_PERIOD;
          period_frac = 0;
        }
        else
        {
          period_frac = (uint32_t)(4294967296.0f*(period - period_int));
        }
        hall_sync_adj = 1;
      }
    }
    else
    {
      if (c == 1)
      {
        /*
          When we got the hall signal, we adjusted pwm interrupt frequency to
          make this interrupt arrive synchronised with that hall signal. Now
          adjust again, trying to hit the following hall signal as closely
          as possible to in (LEDS_TANG-3) interrupts, assuming constant motor
          speed.

          This interrupt occured at time hall_timer. We want to hit
          (prev_hall + prev_hall_period) in (LEDS_TANG-3) interrupts. So each
          interrupt should occur with an interval of
          (prev_hall + prev_hall_period - hall_timer) / (LEDS_TANG-3).
          But cap that reasonably so we do not overflow the 16-bit timer
          registers, nor try to take interrupts far too often, in case of
          glitchy hall readings.
        */
        uint32_t hall_period = prev_hall_period;
        uint32_t target_time = loc_prev_hall + hall_period - hall_timer;
        float period = (float)target_time/(float)(LEDS_TANG-3);
        period_int = (uint32_t)period;
        if (period_int < MIN_PERIOD || period_int > MAX_PERIOD)
        {
          period_int = NOMINAL_PERIOD;
          period_frac = 0;
        }
        else
        {
          period_frac = 4294967296.0f*(period - period_int);
        }
        hall_sync_adj = 0;
      }
      if (c < LEDS_TANG-1)
      {
        generate_scan_counter = scan_counter = c+1;
      }
      else
      {
        if (delayed_hall >= LEDS_TANG*FRAMERATE/4)
        {
          /*
            If we do not receive any hall signal, fall back to using a fixed
            period which approximately matches what the motorcontroller aims
            for.
            This way, we can get reasonable behaviour even without any hall
            sensor or corresponding magnet, though the image will drift
            slowly over time, in the tangential direction.
          */
          period_int = NOMINAL_PERIOD;
          period_frac = 0;
          generate_scan_counter = scan_counter = 0;
        }
        else
        {
          /*
            We reached the end of the frame without getting any hall signal.
            This means that the hall signal is later than we expected, so the
            motor is apparently moving slower than in previous rotation.
            So put in filler scanplanes until next hall signal.
          */
          generate_scan_counter = LEDS_TANG-1;
          ++delayed_hall;
        }
      }
    }

    /*
      Set the period for the next PWM interrupt. Uses the variable
      fractional_period to keep track of the sub-cycle error and correct the
      period adding one occasionally to adjust for rounding error.
    */
    old_fraction = fractional_period;
    new_fraction = old_fraction + period_frac;
    TIM6_ARR = period_int + (new_fraction < old_fraction);
    fractional_period = new_fraction;
    trigger_softint();
    TIM6_SR &= (uint16_t)~TIM_SR_UIF;           /* Clear the interrupt */
  }
}


/*
  Handler for software interrupt.
  This runs at a low priority, and handles generating the next scan plane.
  This allows time-critical processing to interrupt this long-running
  operation.
*/
void
exti0_isr(void)
{
  if (EXTI_PR & EXTI0) {
    uint32_t c = generate_scan_counter;
    uint8_t idx = scanbuffer_idx;
    uint32_t intensity_flag;

    if (c == 0)
    {
      ++frame_counter;
      flip_framebuf();
    }

    intensity_flag = change_intensity_flag;
    if (intensity_flag)
    {
      uint8_t intensity = intensity_flag & 0x7f;
      uint8_t counter;

      /*
        Change LED intensity, by generating new control data instead in the
        coming scanplanes, and shifting it out twice over the next two rounds.
      */
      fill_tlc5955_control_latch((uint8_t *)(scanplane_buffers[idx][0]) + 1,
                                 0, intensity, 4);
      fill_tlc5955_control_latch((uint8_t *)(scanplane_buffers[idx][1]) + 1,
                                 1, intensity, 4);
      fill_tlc5955_control_latch((uint8_t *)(scanplane_buffers[idx][2]) + 1,
                                 2, intensity, 4);
      counter = intensity_flag >> 8;
      /* Generate control data twice, then go back to normal operation. */
      if (counter == 2)
        change_intensity_flag = intensity_flag - 0x100;
      else
        change_intensity_flag = 0;
    }
    else
    {
      make_scan_planes(c, scanplane_buffers[idx][0],
                       scanplane_buffers[idx][1],
                       scanplane_buffers[idx][2]);
    }

    /* Clear the pending interrupt event. */
    EXTI_PR = EXTI0;
  }
}


uint32_t
get_frame_counter(void)
{
  return frame_counter;
}


void
new_intensity(uint8_t intensity)
{
  change_intensity_flag = (2<<8) | intensity;
}


static void
setup_systick(void)
{
  systick_set_reload(0xffffff);
  systick_clear();
  systick_set_clocksource(STK_CSR_CLKSOURCE);
  systick_counter_enable();
}


/*
  Configure a software interrupt to handle the long-running generation of
  next scan plane at a low priority.
*/
static void
setup_softint(void)
{
  /* Software interrupt on EXTI0 (no GPIO triggering). */
  rcc_periph_clock_enable(RCC_SYSCFG);
  /* Disable events on EXTI0. */
  EXTI_EMR &= ~EXTI0;
  /* Disable GPIO triggers. */
  EXTI_RTSR &= ~EXTI0;
  EXTI_FTSR &= ~EXTI0;
  /* Enable interrupts on EXTI0. */
  EXTI_IMR |= EXTI0;

  /* Clear any pending interrupt before enabling. */
  EXTI_PR = EXTI0;
  nvic_set_priority(NVIC_EXTI0_IRQ, 15<<4);
  nvic_enable_irq(NVIC_EXTI0_IRQ);
}


void
setup_timers(void)
{
  setup_systick();
  setup_gsclks();
  setup_scanplane_timer();
  setup_softint();
}
