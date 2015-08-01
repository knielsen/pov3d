#include "ledtorus.h"


struct colour3 {
  uint8_t r, g, b;
};


static inline void
setpix(frame_t *f, uint32_t x, uint32_t y, uint32_t a,
       uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t *p = (*f)[y+x*LEDS_Y+a*(LEDS_Y*LEDS_X)];
  p[0] = r;
  p[1] = g;
  p[2] = b;
}


static struct colour3
hsv2rgb_f(float h, float s, float v)
{
  /* From Wikipedia: https://en.wikipedia.org/wiki/HSL_and_HSV */
  struct colour3 x;
  float c, m, r, g, b;

  c = v * s;
  h *= 6.0f;
  if (h < 1.0f)
  {
    r = c;
    g = c*h;
    b = 0.0f;
  }
  else if (h < 2.0f)
  {
    r = c*(2.0f - h);
    g = c;
    b = 0.0f;
  }
  else if (h < 3.0f)
  {
    r = 0.0f;
    g = c;
    b = c*(h - 2.0f);
  }
  else if (h < 4.0f)
  {
    r = 0.0f;
    g = c*(4.0f - h);
    b = c;
  }
  else if (h < 5.0f)
  {
    r = c*(h - 4.0f);
    g = 0.0f;
    b = c;
  }
  else /* h < 6.0f */
  {
    r = c;
    g = 0.0f;
    b = c*(6.0f - h);
  }
  m = v - c;
  /* Let's try to avoid rounding errors causing under/overflow. */
  x.r = (uint8_t)(0.1f + 255.8f*(r + m));
  x.g = (uint8_t)(0.1f + 255.8f*(g + m));
  x.b = (uint8_t)(0.1f + 255.8f*(b + m));
  return x;
}


static void
cls(frame_t *f)
{
  memset(f, 0, sizeof(frame_t));
}


void
test_img1(void)
{
  uint32_t x, y, a;
  frame_t *f = render_framebuf();

  a = LEDS_TANG / 3;
  cls(f);
  for (a = 0; a < LEDS_TANG; ++a)
  {
    if ((a % 10) > 1)
      continue;
    for (x = 0; x < LEDS_X; ++x)
    {
      for (y = 0; y < LEDS_Y; ++y)
      {
        setpix(f, x, y, a, 128*a/LEDS_TANG, 255*a/LEDS_TANG, 64*a/LEDS_TANG);
      }
    }
  }
}


/*
  Put text at given X-coord, starting at tangential position a.
  Returns value of a following end of text.
  The s_fact is the stretch-factor - tangential pixels per font pixel.
*/
static uint32_t
g_text(frame_t *f, const char *text, uint32_t x, uint32_t a,
       uint8_t c_r, uint8_t c_g, uint8_t c_b, float s_fact)
{
  uint8_t ch;
  float sofar;

  if (s_fact < 0.01f)
    return a;
  /* ToDo: support s_fact < 1. Requires skipping pixels, or merging them. */
  if (s_fact < 1.0f)
    s_fact = 1.0f;
  s_fact = 1.0f / s_fact;
  sofar = s_fact * 0.5f;

  while ((ch = *text++))
  {
    const uint8_t *font_p;
    uint8_t bit_pos;

    if (ch < 32 || ch > 127)
      ch = '?';
    font_p = &tonc_font[8*ch-8*' '];

    bit_pos = 0x80;
    while (bit_pos)
    {
      uint32_t y;
#if LEDS_Y < 8
#error g_text() requires at least 8 pixes in LEDS_Y direction.
#endif
      for (y = 0; y < 8; ++y)
      {
        if (font_p[y] & bit_pos)
          setpix(f, x, y, a, c_r, c_g, c_b);
      }

      if (a == 0)
        a = LEDS_TANG-1;
      else
        --a;
      sofar += s_fact;
      if (sofar >= 1.0f)
      {
        sofar -= 1.0f;
        bit_pos >>= 1;
      }
    }
  }

  return a;
}


static void
envelope(frame_t *f, uint32_t c)
{
  uint32_t a, i;
  uint32_t c2;
  float fact;
  uint8_t c_b;

  c2 = c % 128;
  if (c2 < 32)
    fact = sinf((float)c2 * (F_PI/32.0f));
  else
    fact = 0.0f;
  c_b = (uint32_t)(17.0f + 30.0f*fact);

  for (a = 0; a < LEDS_TANG; ++a)
  {
    setpix(f, 1, 1, a, 0, 0, c_b);
    setpix(f, 1, 6, a, 0, 0, c_b);
    setpix(f, 6, 1, a, 0, 0, c_b);
    setpix(f, 6, 6, a, 0, 0, c_b);
    for (i = 0; i < 4; ++i)
    {
      setpix(f, 0, i+2, a, 0, 0, c_b);
      setpix(f, 6, i+2, a, 0, 0, c_b);
      setpix(f, i+2, 0, a, 0, 0, c_b);
      setpix(f, i+2, 7, a, 0, 0, c_b);
    }
  }
}


void
an_ghost(frame_t *f, uint32_t c, void *st __attribute__((unused)))
{
  uint32_t a, x;
  float ph;
  uint32_t skip;

  ph = (float)c * 0.22f;
  skip = (c % 128) < 64;

  cls(f);
  envelope(f, c);
  for (x = 0; x < LEDS_X; ++x)
  {
    float w = (float)x * 0.31f + ph;
    float y = ((float)LEDS_Y / 2.0f) * (1.0f + cosf(w));
    int32_t i_y = y;

    for (a = 0; a < LEDS_TANG; ++a)
    {
      if (skip && (a % 8))
        continue;
      // ToDo: try some anti-aliasing?
      if (i_y >= 0 && i_y < LEDS_Y)
      {
        struct colour3 col;
        col = hsv2rgb_f((float)a*(1.0f/(float)LEDS_TANG), 0.9f, 0.9f);
        setpix(f, x, i_y, a, col.r, col.g, col.b);
      }
    }
  }
}


static char *
my_str_mk(char *dst, const char *src)
{
  while ((*dst= *src++))
    ++dst;
  return dst;
}

void
an_supply_voltage(frame_t *f, uint32_t c, void *st __attribute__((unused)))
{
  char buf[50];
  static float voltage = 0.0f;
  char *p;
  float stretch;
  uint32_t a, c2;

  cls(f);
  if (voltage == 0.0f || (c%64) == 0)
    voltage = voltage_read();
  p = my_str_mk(buf, "Supply: ");
  float_to_str(p, voltage, 1, 3);
  p = my_str_mk(p+strlen(p), "V");
  c2 = c % 270;
  if (c2 < 72)
    stretch = 1.4f - 0.4f * cosf((float)c2*(2.0f*F_PI/72.0f));
  else
    stretch = 1.0f;
  a = (2*c)%LEDS_TANG;
  g_text(f, buf, 4, a, 255, 100, 20, stretch);
}
