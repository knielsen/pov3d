#include "ledtorus.h"

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
  union {
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
  } u;

  /* Hall sensor on PA3. */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  u.GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  u.GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  u.GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  u.GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  u.GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &u.GPIO_InitStructure);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

  u.TIM_TimeBaseStructure.TIM_Period = 0xffffffff;
  u.TIM_TimeBaseStructure.TIM_Prescaler = 0;
  u.TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  u.TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &u.TIM_TimeBaseStructure);

  u.TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
  u.TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  u.TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  u.TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  /* 0x3 means filter until input is stable for 8 timer clocks. */
  u.TIM_ICInitStructure.TIM_ICFilter = 0x3;
  TIM_ICInit(TIM2, &u.TIM_ICInitStructure);
  TIM_CCxCmd(TIM2, TIM_Channel_4, TIM_CCx_Enable);

  TIM_Cmd(TIM2, ENABLE);

  u.NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
  u.NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  u.NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  u.NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_Init(&u.NVIC_InitStructure);

  TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
}


volatile uint32_t prev_hall = 0;
volatile uint32_t prev_hall_period = 0;

void
TIM2_IRQHandler(void)
{
  uint32_t val;

  val = TIM2->CCR4;               /* Reading CCR4 also clears the interrupt */
  prev_hall_period = val - prev_hall;
  prev_hall = val;
}


uint32_t
check_hall(void)
{
  /* The Hall output is active low. */
  if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) != Bit_RESET)
      return 0;
    else
      return 1;
}


uint32_t
last_hall_period(void)
{
  return prev_hall_period;
}
