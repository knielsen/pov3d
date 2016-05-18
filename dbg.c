/*
  Serial debug output, on PB10.
*/

#include "ledtorus.h"

#include <libopencm3/stm32/usart.h>


void
setup_serial(void)
{
  /* enable peripheral clock for USART3 */
  rcc_periph_clock_enable(RCC_USART3);

  /* GPIOB clock enable */
  rcc_periph_clock_enable(RCC_GPIOB);

  /* GPIOB Configuration:  USART3 TX on PB10. */
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO10);
  gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO10);

  /* Connect USART3 pins to AF7 */
  // TX = PB10
  gpio_set_af(GPIOB, GPIO_AF7, GPIO10);

  usart_set_baudrate(USART3, 115200);
  usart_set_databits(USART3, 8);
  usart_set_stopbits(USART3, USART_STOPBITS_1);
  usart_set_parity(USART3, USART_PARITY_NONE);
  usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART3, USART_MODE_TX);

  usart_enable(USART3);
}


void
serial_putchar(uint32_t c)
{
  usart_send_blocking(USART3, c);
}


void
serial_puts(const char *s)
{
  while (*s)
    serial_putchar((uint8_t)*s++);
}


static void
serial_output_hexdig(uint32_t dig)
{
  serial_putchar((dig >= 10 ? 'A' - 10 + dig : '0' + dig));
}


void
serial_output_hexbyte(uint8_t byte)
{
  serial_output_hexdig(byte >> 4);
  serial_output_hexdig(byte & 0xf);
}


void
println_uint32(uint32_t val)
{
  char buf[13];
  char *p = buf;
  uint32_t l, d;

  l = 1000000000UL;
  while (l > val && l > 1)
    l /= 10;

  do
  {
    d = val / l;
    *p++ = '0' + d;
    val -= d*l;
    l /= 10;
  } while (l > 0);

  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  serial_puts(buf);
}


void
println_int32(int32_t val)
{
  if (val < 0)
  {
    serial_putchar('-');
    println_uint32((uint32_t)0 - (uint32_t)val);
  }
  else
    println_uint32(val);
}


void
print_uint32_hex(uint32_t val)
{
  serial_output_hexbyte(val >> 24);
  serial_output_hexbyte((val >> 16) & 0xff);
  serial_output_hexbyte((val >> 8) & 0xff);
  serial_output_hexbyte(val & 0xff);
}


char *
float_to_str(char *buf, float f, uint32_t dig_before, uint32_t dig_after)
{
  float a;
  uint32_t d;
  uint8_t leading_zero;

  if (f == 0.0f)
  {
    *buf++ = '0';
    *buf = '\0';
    return buf;
  }
  if (f < 0)
  {
    *buf++ = '-';
    f = -f;
  }
  a =  powf(10.0f, (float)dig_before);
  if (f >= a)
  {
    *buf++ = '#';
    *buf = '\0';
    return buf;
  }
  leading_zero = 1;
  while (dig_before)
  {
    a /= 10.0f;
    d = (uint32_t)(f / a);
    if (leading_zero && d == 0 && a >= 10.0f)
      *buf++ = ' ';
    else
    {
      leading_zero = 0;
      *buf++ = '0' + d;
      f -= d*a;
    }
    --dig_before;
  }
  if (!dig_after)
  {
    *buf = '\0';
    return buf;
  }
  *buf++ = '.';
  do
  {
    f *= 10.0f;
    d = (uint32_t)f;
    *buf++ = '0' + d;
    f -= (float)d;
    --dig_after;
  } while (dig_after);
  *buf = '\0';
  return buf;
}


void
println_float(float f, uint32_t dig_before, uint32_t dig_after)
{
  char buf[21];
  char *p = buf;

  float_to_str(p, f, dig_before, dig_after);
  while (*p)
    ++p;
  *p++ = '\r';
  *p++ = '\n';
  *p = '\0';
  serial_puts(buf);
}


void
serial_dump_buf(uint8_t *buf, uint32_t len)
{
  uint32_t i, j;

  for (i = 0; i < len; i += 16)
  {
    print_uint32_hex(i);
    serial_puts(" ");
    for (j = 0; j < 16 && (i+j) < len; ++j)
    {
      if (!(j % 4))
        serial_puts(" ");
      serial_output_hexbyte(buf[i+j]);
    }
    serial_puts("\r\n");
  }
}
