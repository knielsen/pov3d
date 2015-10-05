#include <stdlib.h>

#include "ledtorus.h"


/*
   A factor to compensate that pixels are much smaller in the tangential
   direction than in the raial and vertical direction.
*/
static const float tang_factor = 5.0f;


struct colour3 {
  uint8_t r, g, b;
};


struct hsv3 {
  uint8_t h, s, v;
};
static inline struct hsv3 mk_hsv3(uint8_t h, uint8_t s, uint8_t v)
{
  struct hsv3 res;
  res.h = h;
  res.s = s;
  res.v = v;
  return res;
}


static inline struct hsv3 mk_hsv3_f(float h, float s, float v)
{
  struct hsv3 res;
  res.h = roundf(h*255.0f);
  res.s = roundf(s*255.0f);
  res.v = roundf(v*255.0f);
  return res;
}


/*
  Union with state data for all animations that need one. This way, a single
  statically-allocated memory area can be shared among all animations.
*/
union anim_data {
  struct st_fireworks {
    uint32_t num_phase1;
    uint32_t num_phase2;
    struct {
      float x[3],y[3],z[3],vx,vy,vz,s;
      struct hsv3 col;
      uint32_t base_frame, delay;
      float gl_base, gl_period, gl_amp;
    } p1[25];
    struct {
      float x,y,z,vx,vy,vz,hue;
      struct hsv3 col;
      uint32_t base_frame, delay;
      float fade_factor;
    } p2[300];
  } fireworks;
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


/* Random integer 0 <= x < N. */
static int
irand(int n)
{
  return rand() / (RAND_MAX/n+1);
}

/* Random float 0 <= x <= N. */
static float
drand(float n)
{
  return (float)rand() / ((float)RAND_MAX/n);
}

/* Random unit vector of length a, uniform distribution in angular space. */
static void
vrand(float a, float *x, float *y, float *z)
{
  /*
    Sample a random direction uniformly.

    Uses the fact that cylinder projection of the sphere is area preserving,
    so sample uniformly the cylinder, and project onto the sphere.
  */
  float v = drand(2.0f*F_PI);
  float u = drand(2.0f) - 1.0f;
  float r = sqrtf(1.0f - u*u);
  *x = a*r*cosf(v);
  *y = a*r*sinf(v);
  *z = a*u;
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


static uint32_t
an_ghost(frame_t *f, uint32_t c, union anim_data *data __attribute__((unused)))
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

  return 0;
}


static char *
my_str_mk(char *dst, const char *src)
{
  while ((*dst= *src++))
    ++dst;
  return dst;
}

uint32_t
an_supply_voltage(frame_t *f, uint32_t c, union anim_data *data __attribute__((unused)))
{
  char buf[50];
  static float voltage = 0.0f;
  static uint32_t hall_period = 0;
  char *p;
  float stretch;
  float hue, sat, val;
  uint32_t a, c2;
  struct colour3 textcol;

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
  hue = (float)(c % 512)/512.0f;
  sat = 0.9+0.0999f*sinf((float)(c % 73)*(F_PI*2.0f/73.0f));
  val = 0.85f+0.1499f*sinf((float)(c % 145)*(F_PI*2.0f/145.0f));
  textcol = hsv2rgb_f(hue, sat, val);
  g_text(f, buf, 5, a, textcol.r, textcol.g, textcol.b, stretch);

  if (hall_period == 0 || (c % 25) == 0)
    hall_period = last_hall_period();
  p = my_str_mk(buf, "FPS: ");
  float_to_str(p, 84000000.0f/(float)hall_period, 2, 2);
  g_text(f, buf, 3, 0, 0, 0, 100, 1.0f);

  p = my_str_mk(buf, "IN: ");
  p = float_to_str(p, (float)led_intensity/127.0f*(100.0f-10.0f)+10.0f, 3, 1);
  my_str_mk(p, " %");
  g_text(f, buf, 2, (LEDS_TANG-1) - (c/2)%LEDS_TANG, 100, 0, 0, 1.5f);

  return 0;
}


static uint32_t
in_sdcard(const struct ledtorus_anim *self, union anim_data *data)
{
  const char *filename = self->init_data;

  if (open_file(filename))
    return 1;
  return 0;
}


static uint32_t
an_sdcard(frame_t *f, uint32_t c, union anim_data *data __attribute__((unused)))
{
  int res;

  res = read_sectors((uint32_t *)f, DMA_FRAMEBUF_SIZE/128);
  if (res == 0)
    return 0;
  else
  {
    if (res > 0)
      cls(f);                                   /* Clear on error. */
    /* End animation on EOF or error. */
    return 1;
  }
}


/* Fireworks animation. */

static void
ut_fireworks_shiftem(struct st_fireworks *c, uint32_t i)
{
  uint32_t j;

  for (j = sizeof(c->p1[0].x)/sizeof(c->p1[0].x[0]) - 1; j > 0; --j)
  {
    c->p1[i].x[j] = c->p1[i].x[j-1];
    c->p1[i].y[j] = c->p1[i].y[j-1];
    c->p1[i].z[j] = c->p1[i].z[j-1];
  }
}


static void
ut_fireworks_setpix(frame_t *f, float xf, float yf, float zf, struct hsv3 col)
{
  int x = roundf(xf);
  int y = roundf(yf*tang_factor);
  int z = roundf(zf);
  if (y < 0)
    y += LEDS_TANG;
  else if (y >= LEDS_TANG)
    y -= LEDS_TANG;
  if (x >= 0 && x < LEDS_X && y >= 0 && y < LEDS_TANG && z >= 0 && z < LEDS_Y)
  {
    struct colour3 rgb = hsv2rgb_f((float)col.h*(1.0f/255.0f),
                                   (float)col.s*(1.0f/255.0f),
                                   (float)col.v*(1.0f/255.0f));
    setpix(f, x, (LEDS_Y-1)-z, y, rgb.r, rgb.g, rgb.b);
  }
}


static uint32_t
in_fireworks(const struct ledtorus_anim *self, union anim_data *data)
{
  struct st_fireworks *c = &data->fireworks;
  c->num_phase1 = 0;
  c->num_phase2 = 0;
  return 0;
}


static uint32_t
an_fireworks(frame_t *f, uint32_t frame, union anim_data *data)
{
  uint32_t i, j;

  struct st_fireworks *c= &data->fireworks;

  static const uint32_t max_phase1 = sizeof(c->p1)/sizeof(c->p1[0]);
  static const uint32_t max_phase2 = sizeof(c->p2)/sizeof(c->p2[0]);
  static const float g = 0.045f;
  int new_freq = 25;
  static const float min_height = 4.0f;
  static const float max_height = 7.0f;
  static const uint32_t min_start_delay = 32;
  static const uint32_t max_start_delay = 67;
  static const uint32_t min_end_delay = 50;
  static const uint32_t max_end_delay = 100;
  const float V = 0.5f;
  static const float resist = 0.11f;
  static const float min_fade_factor = 0.22f/15.0f;
  static const float max_fade_factor = 0.27f/15.0f;
  float joy_angle, joy_magnitude;
  float joy_val;
  float global_hue = -1.0f;
  float global_sat = 0.85f;
  float hue_max = 1.1f;

  joy_angle = joy_r_angle_mag(&joy_magnitude);
  if (joy_magnitude > 0.8f)
    global_hue = joy_angle * (1.0f/(2.0f*F_PI));
  joy_val = joy_l_vert();
  if (joy_val > 0.5f)
    global_sat = joy_val;
  else if (joy_val < -0.5f)
    global_sat = joy_val + 1.0f;
  if (key_cross_state())
  {
    new_freq /= 4;
    hue_max += 5.0f;
  }

  /* Start a new one occasionally. */
  if (c->num_phase1 == 0 || (c->num_phase1 < max_phase1 && irand(new_freq) == 0))
  {
    i = c->num_phase1++;

    c->p1[i].x[0] = (float)(LEDS_X-1)/2.0f - 1.2f + drand(2.4f);
    c->p1[i].y[0] = drand((float)LEDS_TANG/tang_factor);
    c->p1[i].z[0] = 0.0f;
    for (j = 0; j < sizeof(c->p1[0].x)/sizeof(c->p1[0].x[0]) - 1; ++j)
      ut_fireworks_shiftem(c, i);

    c->p1[i].vx = drand(0.35f) - 0.175f;
    c->p1[i].vy = drand(0.35f/tang_factor) - 0.175f/tang_factor;
    c->p1[i].s = min_height + drand(max_height - min_height);
    c->p1[i].vz = sqrt(2*g*c->p1[i].s);
    c->p1[i].col = mk_hsv3_f(0.8f, 0.0f, 0.5f);
    c->p1[i].base_frame = frame;
    c->p1[i].delay = min_start_delay + irand(max_start_delay - min_start_delay);
    c->p1[i].gl_base = frame;
    c->p1[i].gl_period = 0;
  }

  for (i = 0; i < c->num_phase1; )
  {
    uint32_t d = frame - c->p1[i].base_frame;
    if (d < c->p1[i].delay)
    {
      /* Waiting for launch - make fuse glow effect. */
      uint32_t gl_delta = frame - c->p1[i].gl_base;
      if (gl_delta >= c->p1[i].gl_period)
      {
        c->p1[i].gl_base = frame;
        c->p1[i].gl_period = 8 + irand(6);
        c->p1[i].gl_amp = 0.7f + drand(0.3f);
        gl_delta = 0;
      }
      float glow = c->p1[i].gl_amp*sin((float)gl_delta/c->p1[i].gl_period*F_PI);
      c->p1[i].col = mk_hsv3_f(0.8f, 0.0f, 0.44f + 0.31f*glow);
      ++i;
    }
    else if (c->p1[i].z[0] > c->p1[i].s)
    {
      /* Kaboom! */
      /* Delete this one, and create a bunch of phase2 ones (if room). */
      int k = 10 + irand(20);
      float common_hue = global_hue >= 0.0f ? global_hue : drand(hue_max);
      while (k-- > 0)
      {
        if (c->num_phase2 >= max_phase2)
          break;            /* No more room */
        j = c->num_phase2++;

        /* Sample a random direction uniformly. */
        float vx;
        float vy;
        float vz;
        vrand(V, &vx, &vy, &vz);

        c->p2[j].x = c->p1[i].x[0];
        c->p2[j].y = c->p1[i].y[0];
        c->p2[j].z = c->p1[i].z[0];
        c->p2[j].vx = c->p1[i].vx + vx;
        c->p2[j].vy = c->p1[i].vy + vy;
        c->p2[j].vz = c->p1[i].vz + vz;
        c->p2[j].hue = common_hue <= 1.0f? common_hue : drand(1.0f);
        c->p2[j].col = mk_hsv3_f(c->p2[j].hue, global_sat, 1.0f);
        c->p2[j].base_frame = frame;
        c->p2[j].delay = min_end_delay + irand(max_end_delay - min_end_delay);
        c->p2[j].fade_factor =
          min_fade_factor + drand(max_fade_factor - min_fade_factor);
      }
      c->p1[i] = c->p1[--c->num_phase1];
    }
    else
    {
      ut_fireworks_shiftem(c, i);
      c->p1[i].col = mk_hsv3_f(0.8f, 0.0f, 0.75f);
      c->p1[i].x[0] += c->p1[i].vx;
      c->p1[i].y[0] += c->p1[i].vy;
      c->p1[i].z[0] += c->p1[i].vz;
      c->p1[i].vz -= g;
      ++i;
    }
  }

  for (i = 0; i < c->num_phase2;)
  {
    c->p2[i].x += c->p2[i].vx;
    c->p2[i].y += c->p2[i].vy;
    c->p2[i].z += c->p2[i].vz;

    c->p2[i].vx -= resist*c->p2[i].vx;
    c->p2[i].vy -= resist*c->p2[i].vy;
    c->p2[i].vz -= resist*c->p2[i].vz + g;

    float value = 1.0f - c->p2[i].fade_factor*(frame - c->p2[i].base_frame);
    if (value < 0.0f)
      value = 0.0f;
    c->p2[i].col = mk_hsv3_f(c->p2[i].hue, global_sat, value);

    if (c->p2[i].z <= 0.0f)
    {
      c->p2[i].z = 0.0f;
      if (c->p2[i].delay-- == 0 || value <= 0.05f)
      {
        /* Delete it. */
        c->p2[i] = c->p2[--c->num_phase2];
      }
      else
        ++i;
    }
    else
      ++i;
  }

  cls(f);
  /* Mark out the "ground". */
  for (i = 0; i < LEDS_TANG; ++i)
    for (j = 2; j < LEDS_X-2; ++j)
      setpix(f, j, LEDS_Y-1, i, 0, 0, 17);

  /*
    Draw stage2 first, so we don't overwrite a new rocket with an old, dark
    ember.
  */
  for (i = 0; i < c->num_phase2; ++i)
    ut_fireworks_setpix(f, c->p2[i].x, c->p2[i].y, c->p2[i].z, c->p2[i].col);
  for (i = 0; i < c->num_phase1; ++i)
  {
    for (j = 0; j < sizeof(c->p1[0].x)/sizeof(c->p1[0].x[0]); ++j)
      ut_fireworks_setpix(f, c->p1[i].x[j], c->p1[i].y[j], c->p1[i].z[j], c->p1[i].col);
  }

  return 0;
}


const struct ledtorus_anim anim_table[] = {
  { "Status",
    "Couple of scroll-texts displaying status",
    512, NULL, NULL, an_supply_voltage },

  { "Ghost",
    "Animated cosine-wave",
    512, NULL, NULL, an_ghost },

  { "Simplex1",
    "First prototype Simplex Noise animation",
    0, "SIMPLEX1.P3D", in_sdcard, an_sdcard },

  { "Simplex2",
    "Second prototype Simplex Noise animation",
    0, "SIMPLEX2.P3D", in_sdcard, an_sdcard },

  { "Fireworks",
    "Fireworks animation",
    1024, NULL, in_fireworks, an_fireworks },
};
const uint32_t anim_table_length = sizeof(anim_table)/sizeof(anim_table[0]);


static union anim_data shared_anim_data;

uint32_t
anim_init(uint32_t anim_idx)
{
  if (!anim_table[anim_idx].init)
    return 0;
  return (*anim_table[anim_idx].init)(&anim_table[anim_idx], &shared_anim_data);
}


uint32_t
anim_nextframe(uint32_t anim_idx, frame_t *f, uint32_t anim_state)
{
  return (*anim_table[anim_idx].nextframe)(f, anim_state, &shared_anim_data);
}
