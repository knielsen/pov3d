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
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Hall sensor on PA3. */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
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
