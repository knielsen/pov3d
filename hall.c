#include "ledtorus.h"

#include <libopencm3/cm3/nvic.h>


/*
  Hall sensor.

  The Hall sensor is an Infineon TLE4906L.
  It pulls its output low when it senses the magnetic field.
  There is a pull-up on the board that pulls the output high when no
  magnetic field is present.

  The Hall sensor output is connected to pin PA3.
*/

void
setup_hall(void)
{
  /* Hall sensor on PA3. */
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO3);

  rcc_periph_clock_enable(RCC_TIM2);
  gpio_set_af(GPIOA, GPIO_AF1, GPIO3);

  timer_set_period(TIM2, 0xffffffff);
  timer_set_prescaler(TIM2, 0);
  timer_set_clock_division(TIM2, 0);
  timer_direction_up(TIM2);

  timer_ic_disable(TIM2, TIM_IC4);
  timer_ic_set_polarity(TIM2, TIM_IC4, TIM_IC_FALLING);
  /* TIM_IC_IN_TI4 selects TIM2_CH4 as the input for capture-compare. */
  timer_ic_set_input(TIM2, TIM_IC4, TIM_IC_IN_TI4);
  timer_ic_set_prescaler(TIM2, TIM_IC4, TIM_IC_PSC_OFF);
  timer_ic_set_filter(TIM2, TIM_IC4, TIM_IC_CK_INT_N_8);
  timer_ic_enable(TIM2, TIM_IC4);

  timer_enable_counter(TIM2);

  nvic_set_priority(NVIC_TIM2_IRQ, 5<<4);
  nvic_enable_irq(NVIC_TIM2_IRQ);

  timer_enable_irq(TIM2, TIM_DIER_CC4IE);
}


volatile uint32_t prev_hall = 0;
volatile uint32_t prev_hall_period = 0;

#ifdef DEBUG_SPEED_STABILITY
/*
  A buffer in the otherwise unused core-coupled SRAM.
  In this buffer, we store the measured period of each rotation.
  This can then be dumped with GDB, to debug motor speed stability.
*/
uint32_t *const prev_hall_period_buffer = (void *)0x10000000;
uint32_t prev_hall_period_ptr = 0;
#endif

void
tim2_isr(void)
{
  uint32_t val;

  val = TIM2_CCR4;                /* Reading CCR4 also clears the interrupt */
  prev_hall_period = val - prev_hall;
  prev_hall = val;
#ifdef DEBUG_SPEED_STABILITY
  if (prev_hall_period_ptr < 16384)
    prev_hall_period_buffer[prev_hall_period_ptr++] = prev_hall_period;
#endif
}


uint32_t
check_hall(void)
{
  /* The Hall output is active low. */
  if (gpio_get(GPIOA, GPIO3))
      return 0;
    else
      return 1;
}


uint32_t
last_hall_period(void)
{
  return prev_hall_period;
}
