#include "ledtorus.h"

void delay(__IO uint32_t nCount)
{
    while(nCount--)
        __asm("nop"); // do nothing
}
