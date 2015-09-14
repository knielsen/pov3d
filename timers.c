/*
  Timers.

  We use TIM5 ch1-3 (on PA0-2) for GSCLKs.

  TIM5 is on the 42 MHz bus, so max timer frequency is 84 MHz.
  To get 50% duty cycle (and for GPIO to keep up?), max GSCLK is then 21 MHz.
  This is obtained with a PWM period of 4.
*/

#include "ledtorus.h"


static volatile uint32_t scan_counter;

#define NOMINAL_PERIOD ((MCU_HZ/2 + (LEDS_TANG*FRAMERATE/2)) / (LEDS_TANG*FRAMERATE))
/* Set limits so that if Hall is completely off we can ignore it. */
#define MIN_PERIOD (NOMINAL_PERIOD*8/10)
#define MAX_PERIOD 64000


static void
setup_gsclks(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);

  TIM_TimeBaseStructure.TIM_Period = GSCLK_PERIOD-1;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = GSCLK_PERIOD/2;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM5, &TIM_OCInitStructure);
  TIM_OC2Init(TIM5, &TIM_OCInitStructure);
  TIM_OC3Init(TIM5, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM5, ENABLE);
  TIM_Cmd(TIM5, ENABLE);
}


static void
setup_scanplane_timer(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

  TIM_TimeBaseStructure.TIM_Period = NOMINAL_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  /* Remaining fields not used for TIM6/TIM7. */
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

  TIM_ARRPreloadConfig(TIM6, ENABLE);
  TIM_SelectOnePulseMode(TIM6, TIM_OPMode_Repetitive);
  TIM_UpdateRequestConfig(TIM6, TIM_UpdateSource_Regular);
  TIM_UpdateDisableConfig(TIM6, DISABLE);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
  NVIC_Init(&NVIC_InitStructure);
  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

  TIM_Cmd(TIM6, ENABLE);
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
  EXTI->PR = EXTI_Line0;
  EXTI->SWIER = EXTI_Line0;
}


static uint32_t scanplane_buffers[2][3][25];
static uint8_t scanbuffer_idx = 0;
static uint8_t init_counter = 0;
static volatile uint32_t frame_counter = 0;
static volatile uint32_t change_intensity_flag = 0;

void TIM6_DAC_IRQHandler(void)
{
  if (TIM6->SR & TIM_IT_Update)
  {
    uint32_t hall_timer, c, loc_prev_hall;
    uint8_t idx;
    uint8_t ic;
    uint8_t new_round;
    static uint8_t saved_new_round = 0;
    static uint32_t saved_prev_hall = 0;

    ic = init_counter;
    if (ic >= 2)
      latch_scanplanes();
    hall_timer = current_hall_timer();

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
    if (c == 0)
      saved_new_round = new_round = 1;
    else
      new_round = saved_new_round;
    loc_prev_hall = prev_hall;
    if (new_round && loc_prev_hall != saved_prev_hall)
    {
      /*
        Adjust timer period as needed to remain fixed with respect to the
        Hall sensor trigger.
      */
      uint32_t hall_period, remain_scanplanes, remain_time;
      uint32_t new_period;

      hall_period = prev_hall_period;
      remain_scanplanes = (LEDS_TANG-1) - c;
      if (remain_scanplanes == 0)
      {
        remain_scanplanes = LEDS_TANG;
        remain_time = hall_period;
      }
      else
        remain_time = loc_prev_hall + hall_period - hall_timer;
      new_period = (remain_time + remain_scanplanes/2) / remain_scanplanes;
      if (new_period < MIN_PERIOD || new_period > MAX_PERIOD)
        new_period = NOMINAL_PERIOD;
      TIM6->ARR = new_period;

      saved_new_round = 0;
      saved_prev_hall = loc_prev_hall;
    }

    /*
      Now do the rest of the processing (preparation of the next scanplane to
      shift out to TLCs) in a low-priority software interrupt, allowing more
      time-critical stuff to interrupt it.
    */
    trigger_softint();

    TIM6->SR = (uint16_t)~TIM_IT_Update;
  }
}


/*
  Handler for software interrupt.
  This runs at a low priority, and handles generating the next scan plane.
  This allows time-critical processing to interrupt this long-running
  operation.
*/
void
EXTI0_IRQHandler(void)
{
  if (EXTI->PR & EXTI_Line0) {
    uint32_t c = scan_counter;
    uint8_t idx = scanbuffer_idx;
    uint32_t intensity_flag;

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
    ++c;
    if (c >= LEDS_TANG)
    {
      c = 0;
      ++frame_counter;
      flip_framebuf();
    }
    scan_counter = c;

    /* Clear the pending interrupt event. */
    EXTI->PR = EXTI_Line0;
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
  SysTick->LOAD = 0xffffff;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}


/*
  Configure a software interrupt to handle the long-running generation of
  next scan plane at a low priority.
*/
static void
setup_softint(void)
{
  union {
    NVIC_InitTypeDef NVIC_InitStruct;
  } u;

  /* Software interrupt on EXTI0 (no GPIO triggering). */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Disable events on EXTI0. */
  EXTI->EMR &= ~EXTI_Line0;
  /* Disable GPIO triggers. */
  EXTI->RTSR &= ~EXTI_Line0;
  EXTI->FTSR &= ~EXTI_Line0;
  /* Enable interrupts on EXTI0. */
  EXTI->IMR |= EXTI_Line0;

  /* Clear any pending interrupt before enabling. */
  EXTI->PR = EXTI_Line0;
  u.NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
  u.NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 15;
  u.NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  u.NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&u.NVIC_InitStruct);
}


void
setup_timers(void)
{
  setup_systick();
  setup_gsclks();
  setup_scanplane_timer();
  setup_softint();
}
