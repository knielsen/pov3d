#include "ledtorus.h"

void delay(uint32_t nCount)
{
  do
  {
    __asm __volatile("");  // Do nothing but prevent optimising away the loop
    --nCount;
  } while (nCount > 0);
}
