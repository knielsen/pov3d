/*
  Timers.

  We use TIM5 ch1-3 (on PA0-2) for GSCLKs.

  TIM5 is on the 42 MHz bus, so max timer frequency is 84 MHz.
  To get 50% duty cycle (and for GPIO to keep up?), max GSCLK is then 21 MHz.
  This is obtained with a PWM period of 4.
*/

#include "pov3d.h"


static volatile uint32_t scan_counter;


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

  TIM_TimeBaseStructure.TIM_Period = 4095;
  TIM_TimeBaseStructure.TIM_Prescaler = GSCLK_PERIOD-1;
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
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
  NVIC_Init(&NVIC_InitStructure);
  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

  TIM_Cmd(TIM6, ENABLE);
}


static uint32_t scanplane_buffers[2][3][25];
static uint8_t scanbuffer_idx = 0;
static uint8_t init_counter = 0;
static volatile uint32_t frame_counter = 0;
void TIM6_DAC_IRQHandler(void)
{
  if (TIM6->SR & TIM_IT_Update)
  {
    uint32_t c;
    uint8_t idx;
    uint8_t ic;

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

    /* ToDo: Just trigger a software interrupt to do this at low prio. */
    make_scan_planes(c, scanplane_buffers[idx][0],
                     scanplane_buffers[idx][1],
                     scanplane_buffers[idx][2]);

    ++c;
    if (c >= LEDS_TANG)
    {
      c = 0;
      ++frame_counter;
      flip_framebuf();
    }
    scan_counter = c;

    TIM6->SR = (uint16_t)~TIM_IT_Update;
  }
}


uint32_t
get_frame_counter(void)
{
  return frame_counter;
}


static void
setup_systick(void)
{
  SysTick->LOAD = 0xffffff;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}


void
setup_timers(void)
{
  setup_systick();
  setup_gsclks();
  setup_scanplane_timer();
}
