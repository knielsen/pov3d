#include <ledtorus.h>

/*
  Gamma correction, mapping frame buffer intensity 0..255 into
  12-bit liniar gray-scale PWM-value 0..4095.

  This table was obtained with:

    perl -le 'print 0; print 1+int(0.5+4094*($_/255)**1.78) for (1..255);'
*/
static const uint16_t gammas_flash[256] = {
     0,    1,    2,    3,    4,    5,    6,    8,
    10,   12,   14,   16,   19,   21,   24,   27,
    31,   34,   38,   41,   45,   49,   53,   58,
    62,   67,   71,   76,   81,   86,   92,   97,
   103,  109,  114,  120,  127,  133,  139,  146,
   152,  159,  166,  173,  180,  188,  195,  203,
   210,  218,  226,  234,  243,  251,  259,  268,
   277,  285,  294,  303,  313,  322,  331,  341,
   351,  360,  370,  380,  390,  401,  411,  421,
   432,  443,  454,  465,  476,  487,  498,  509,
   521,  533,  544,  556,  568,  580,  592,  605,
   617,  630,  642,  655,  668,  681,  694,  707,
   720,  734,  747,  761,  775,  788,  802,  816,
   831,  845,  859,  874,  888,  903,  918,  932,
   947,  963,  978,  993, 1008, 1024, 1040, 1055,
  1071, 1087, 1103, 1119, 1135, 1152, 1168, 1185,
  1201, 1218, 1235, 1252, 1269, 1286, 1303, 1321,
  1338, 1356, 1373, 1391, 1409, 1427, 1445, 1463,
  1481, 1500, 1518, 1537, 1555, 1574, 1593, 1612,
  1631, 1650, 1669, 1689, 1708, 1728, 1747, 1767,
  1787, 1807, 1827, 1847, 1867, 1887, 1908, 1928,
  1949, 1970, 1990, 2011, 2032, 2053, 2074, 2096,
  2117, 2138, 2160, 2182, 2203, 2225, 2247, 2269,
  2291, 2313, 2336, 2358, 2381, 2403, 2426, 2449,
  2471, 2494, 2517, 2541, 2564, 2587, 2611, 2634,
  2658, 2681, 2705, 2729, 2753, 2777, 2801, 2825,
  2850, 2874, 2899, 2923, 2948, 2973, 2998, 3023,
  3048, 3073, 3098, 3123, 3149, 3174, 3200, 3226,
  3251, 3277, 3303, 3329, 3356, 3382, 3408, 3434,
  3461, 3488, 3514, 3541, 3568, 3595, 3622, 3649,
  3676, 3704, 3731, 3758, 3786, 3814, 3841, 3869,
  3897, 3925, 3953, 3981, 4010, 4038, 4066, 4095
};
/*
  SRAM copy for fast access.
  Additionally, the values are byte-swapped to match the big-endian
  SPI used by the TLC5955.
*/
static uint16_t gammas[256];

/*
  Angular offsets of the different LEDs.
  The unit is turns [0..1[.
  This table is copied into SRAM for fast access, and scaled to the number
  of pixels in the tangential direction.

  A value of 0 marks a not present LED.

  The table is indexed as Y+8*X (since multiply by 8 is more efficient than
  by 7).
*/
static const float led_angles_flash[7*8] = {
  0.0f, 0.0f, 0.3631262f, 0.4593547f, 0.5406453f, 0.6368738f, 0.0f, 0.0f,
  0.0f, 0.3179607f, 0.4080512f, 0.4708647f, 0.5291353f, 0.5919488f, 0.6820393f, 0.0f,
  0.2641870f, 0.3740195f, 0.4298081f, 0.4772767f, 0.5227233f, 0.5701919f, 0.6259805f, 0.7358130f,
  0.3476761f, 0.4007556f, 0.4430292f, 0.4813702f, 0.5186298f, 0.5569708f, 0.5992444f, 0.6523239f,
  0.3780867f, 0.4175499f, 0.4519883f, 0.4842119f, 0.5157881f, 0.5480117f, 0.5824501f, 0.6219133f,
  0.3972262f, 0.4292826f, 0.4584827f, 0.4863004f, 0.5136996f, 0.5415173f, 0.5707174f, 0.6027738f,
  0.f, 0.4380048f, 0.4634151f, 0.4879005f, 0.5120995f, 0.5365849f, 0.5619952f, 0.0f
};
/*
  SRAM copy for fast access.
  Also transposed by tlc_map_flash[] for direct access by TLC output.
  And scaled to LEDS_TANG.

  So led_angles[T][15-N] gives the angle offset of TLC outputs [RGB]N, in
  units of 1/LEDS_TANG turns.
*/
static uint16_t led_angles[3][16];


/*
  Mapping from (Y+X*8) to distance-to-center. 0.0f for a non-present LED.
  No SRAM version, as we do not need speed for lookups - it is only used
  to adjust DC value to correct brightness for longer sweeps of outer LEDs.
*/
static const float led_center_distances_flash[7*8] = {
   0.0f,   0.0f,  14.19,  14.19,  14.19,  14.19,   0.0f,   0.0f,
   0.0f,  19.69,  19.69,  19.69,  19.69,  19.69,  19.69,   0.0f,
  25.19,  25.19,  25.19,  25.19,  25.19,  25.19,  25.19,  25.19,
  30.69,  30.69,  30.69,  30.69,  30.69,  30.69,  30.69,  30.69,
  36.19,  36.19,  36.19,  36.19,  36.19,  36.19,  36.19,  36.19,
  41.69,  41.69,  41.69,  41.69,  41.69,  41.69,  41.69,  41.69,
   0.0f,  47.19,  47.19,  47.19,  47.19,  47.19,  47.19,   0.0f
};


/*
  Table mapping inverse LED number to offset (Y+8*X) into scan plane.
  This gives a way to go from TLC index (order of shift-out) to position in
  scan plane (or led_angles_flash[] array).

  The first shifted-out outputs are B15, G15, R15, ... down to R0.
     0   8   16  24  32  40  48
0    #   #   B4  B1  A12 A9   #
1    #  B0   B2  A14 A8  A4  A0
2   B5  B6   A10 A13 A5  A1  A2
3   B3  B7   A15 A11 A7  A3  A6
4   B15 B11  C3  C7  C11 C15 C10
5   B9  B10  C6  C1  C9  C13 C14
6    #  B12  B14 C2  C4  C8  C12
7    #   #   B8  B13 C0  C5   #
*/
static const uint16_t tlc_map_flash[3][16] = {
  { 19, 25, 26, 32, 27, 18, 40, 33, 35, 51, 34, 41, 43, 50, 42, 49 },
  { 4, 22, 31, 14, 12, 13, 5, 23, 11, 10, 2, 16, 3, 17, 24, 9 },
  { 44, 53, 45, 54, 36, 52, 37, 46, 28, 21, 47, 38, 20, 30, 29, 39 }
};
static uint16_t tlc_map[3][16];


/*
  The double framebuffers.
  Align on a 32-bit boundary, and pad to a multiple of 512 bytes (SD-card
  sector size) to allow direct loading by DMA.
*/
static uint32_t dma_able_framebuf[2][DMA_FRAMEBUF_SIZE];
static uint8_t render_idx;


static inline uint8_t
(*get_framebuf(uint8_t idx))[LEDS_Y*LEDS_X*LEDS_TANG][3]
{
  return (uint8_t (*)[LEDS_Y*LEDS_X*LEDS_TANG][3])dma_able_framebuf[idx];
}


/*
  This is the central routine that takes an x/y plane in the framebuffer
  and populates three buffers with corresponding RGB grayscale values for
  the TLC5955 LED drivers.
*/
void
make_scan_planes(uint32_t angle,
                 uint32_t * __restrict b1,
                 uint32_t * __restrict b2,
                 uint32_t * __restrict b3)
{
  uint16_t *p1 = (uint16_t *)b1;
  uint16_t *p2 = (uint16_t *)b2;
  uint16_t *p3 = (uint16_t *)b3;
  uint16_t *ps[3] = { p1, p2, p3 };
  uint8_t (*f)[LEDS_Y*LEDS_X*LEDS_TANG][3] = get_framebuf(1 - render_idx);
  uint32_t t, i;

  for (t = 0; t < 3; ++t)
  {
    uint16_t *map = &tlc_map[t][0];
    uint16_t *angles = &led_angles[t][0];
    uint16_t *p = ps[t];
    *p++ = 0;
    for (i = 0; i < 16; ++i)
    {
      uint8_t c_r, c_g, c_b;
      uint8_t (*pix_ptr)[3];
      uint32_t idx = map[i];
      uint32_t angle_offset = angles[i];
      uint32_t a = angle + angle_offset;
      if (a >= LEDS_TANG)
        a -= LEDS_TANG;
      pix_ptr = &(*f)[a*(LEDS_X*LEDS_Y) + idx];
      c_b = (*pix_ptr)[2];
      *p++ = gammas[c_b];
      c_g = (*pix_ptr)[1];
      *p++ = gammas[c_g];
      c_r = (*pix_ptr)[0];
      *p++ = gammas[c_r];
    }
  }
}


void
init_tlc(void)
{
  uint32_t i, j;

  /*
    Compute the gamma corrections.

    We multiply by 16 to use the full 0..65535 range, because this is needed
    with enhanced spectrum PWM.

    We byteswap as Cortex M4 is little endian, while the TLC5955's are big
    endian.
  */
  for (i = 0; i < 256; ++i)
    gammas[i] = __REV16(gammas_flash[i]*16);

  for (j = 0; j < 3; ++j)
  {
    for (i = 0; i < 16; ++i)
    {
      uint32_t idx;
      float angle;

      idx = tlc_map_flash[j][i];
      tlc_map[j][i] = idx;
      angle = led_angles_flash[idx];
      led_angles[j][i] = (uint16_t)((float)LEDS_TANG * angle);
    }
  }
}


float
led_distance_to_center_xy(uint32_t x, uint32_t y)
{
  return led_center_distances_flash[y+x*8];
}


float
led_distance_to_center_tlc(uint32_t tlc, uint32_t output)
{
  return led_center_distances_flash[tlc_map_flash[tlc][output]];
}


uint8_t (*render_framebuf(void))[LEDS_Y*LEDS_X*LEDS_TANG][3]
{
  return get_framebuf(render_idx);
}


uint8_t (*display_framebuf(void))[LEDS_Y*LEDS_X*LEDS_TANG][3]
{
  return get_framebuf(1-render_idx);
}


void
flip_framebuf(void)
{
  render_idx = 1 - render_idx;
}


uint8_t led_intensity = 50;

void
led_decrease_intensity(void)
{
  if (led_intensity > 0)
  {
    uint8_t intensity_dec = (led_intensity > 15 ? 10 : 2);
    if (led_intensity > intensity_dec)
      led_intensity -= intensity_dec;
    else
      led_intensity = 0;
    new_intensity(led_intensity);
  }
}


void
led_increase_intensity(void)
{
  if (led_intensity < 127)
  {
    uint8_t intensity_inc = (led_intensity > 24 ? 10 : 2);
    if ((127 - led_intensity) > intensity_inc)
      led_intensity += intensity_inc;
    else
      led_intensity = 127;
    new_intensity(led_intensity);
  }
}
