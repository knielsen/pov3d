#include <ledtorus.h>

/*
  Define this for motors that are designed for counter-clockwise spin
  direction.
*/
//#define COUNTER_CLOCKWISE

#define SPIS 6

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

  The table is indexed as Y+16*X (since multiply by 16 is more efficient than
  by 14).
*/
static const float led_angles_flash[LEDS_X*LEDS_Y] = {
  0.0f, 0.0f, 0.0f, 0.0f, 0.2980995f, 0.3805741f, 0.4329130f, 0.4782262f, 0.5217738f, 0.5670870f, 0.6194259f, 0.7019005f, 0.0f, 0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 0.0f, 0.0f, 0.3551837f, 0.4046519f, 0.4450682f, 0.4820125f, 0.5179875f, 0.5549318f, 0.5953481f, 0.6448163f, 0.0f, 0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 0.0f, 0.3335798f, 0.3825087f, 0.4201923f, 0.4534356f, 0.4846754f, 0.5153246f, 0.5465644f, 0.5798077f, 0.6174913f, 0.6664202f, 0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 0.3134574f, 0.3640599f, 0.4002630f, 0.4312094f, 0.4595653f, 0.4866506f, 0.5133494f, 0.5404347f, 0.5687906f, 0.5997370f, 0.6359401f, 0.6865426f, 0.0f, 0.0f,
  0.0f, 0.2922072f, 0.3479355f, 0.3835527f, 0.4130335f, 0.4394784f, 0.4642562f, 0.4881744f, 0.5118256f, 0.5357438f, 0.5605216f, 0.5869665f, 0.6164473f, 0.6520645f, 0.7077928f, 0.0f,
  0.2543407f, 0.3332318f, 0.3690456f, 0.3976280f, 0.4227586f, 0.4459342f, 0.4679650f, 0.4893857f, 0.5106143f, 0.5320350f, 0.5540658f, 0.5772414f, 0.6023720f, 0.6309544f, 0.6667682f, 0.7456593f,
  0.3192288f, 0.3560840f, 0.3842062f, 0.4084303f, 0.4304515f, 0.4511239f, 0.4709725f, 0.4903718f, 0.5096282f, 0.5290275f, 0.5488761f, 0.5695485f, 0.5915697f, 0.6157938f, 0.6439160f, 0.6807712f,
  0.3442042f, 0.3722474f, 0.3958699f, 0.4170471f, 0.4367077f, 0.4553917f, 0.4734613f, 0.4911902f, 0.5088098f, 0.5265387f, 0.5446083f, 0.5632923f, 0.5829529f, 0.6041301f, 0.6277526f, 0.6557958f,
  0.3613858f, 0.3846537f, 0.4052177f, 0.4241110f, 0.4419051f, 0.4589659f, 0.4755554f, 0.4918803f, 0.5081197f, 0.5244446f, 0.5410341f, 0.5580949f, 0.5758890f, 0.5947823f, 0.6153463f, 0.6386142f,
  0.3744797f, 0.3946131f, 0.4129217f, 0.4300230f, 0.4462969f, 0.4620045f, 0.4773422f, 0.4924701f, 0.5075299f, 0.5226578f, 0.5379955f, 0.5537031f, 0.5699770f, 0.5870783f, 0.6053869f, 0.6255203f,
  0.3849793f, 0.4028468f, 0.4194037f, 0.4350524f, 0.4500601f, 0.4646205f, 0.4788848f, 0.4929800f, 0.5070200f, 0.5211152f, 0.5353795f, 0.5499399f, 0.5649476f, 0.5805963f, 0.5971532f, 0.6150207f,
  0.3936699f, 0.4097994f, 0.4249462f, 0.4393886f, 0.4533228f, 0.4668969f, 0.4802303f, 0.4934251f, 0.5065749f, 0.5197697f, 0.5331031f, 0.5466772f, 0.5606114f, 0.5750538f, 0.5902006f, 0.6063301f,
  0.4010244f, 0.4157664f, 0.4297474f, 0.4431690f, 0.4561799f, 0.4688963f, 0.4814141f, 0.4938172f, 0.5061828f, 0.5185859f, 0.5311037f, 0.5438201f, 0.5568310f, 0.5702526f, 0.5842336f, 0.5989756f,
  0.4073526f, 0.4209543f, 0.4339517f, 0.4464962f, 0.4587035f, 0.4706666f, 0.4824640f, 0.4941652f, 0.5058348f, 0.5175360f, 0.5293334f, 0.5412965f, 0.5535038f, 0.5660483f, 0.5790457f, 0.5926474f
};
/*
  SRAM copy for fast access.
  Also transposed by tlc_map_flash[] for direct access by TLC output.
  And scaled to LEDS_TANG.

  So led_angles[T][15-N] gives the angle offset of TLC outputs [RGB]N, in
  units of 1/LEDS_TANG turns.
*/
static uint16_t led_angles[SPIS][32];


/*
  Mapping from (Y+X*8) to distance-to-center. 0.0f for a non-present LED.
  No SRAM version, as we do not need speed for lookups - it is only used
  to adjust DC value to correct brightness for longer sweeps of outer LEDs.
*/
static const float led_center_distances_flash[LEDS_X*LEDS_Y] = {
    0.0f,  0.0f,   0.0f,   0.0f,  19.11f, 19.11f, 19.11f, 19.11f, 19.11f, 19.11f, 19.11f, 19.11f,  0.0f,   0.0f,   0.0f,   0.0f,
    0.0f,  0.0f,   0.0f,   0.0f,  23.11f, 23.11f, 23.11f, 23.11f, 23.11f, 23.11f, 23.11f, 23.11f,  0.0f,   0.0f,   0.0f,   0.0f,
    0.0f,  0.0f,   0.0f,  27.11f, 27.11f, 27.11f, 27.11f, 27.11f, 27.11f, 27.11f, 27.11f, 27.11f, 27.11f,  0.0f,   0.0f,   0.0f,
    0.0f,  0.0f,  31.11f, 31.11f, 31.11f, 31.11f, 31.11f, 31.11f, 31.11f, 31.11f, 31.11f, 31.11f, 31.11f, 31.11f,  0.0f,   0.0f,
    0.0f, 35.11f, 35.11f, 35.11f, 35.11f, 35.11f, 35.11f, 35.11f, 35.11f, 35.11f, 35.11f, 35.11f, 35.11f, 35.11f, 35.11f,  0.0f,
  39.11f, 39.11f, 39.11f, 39.11f, 39.11f, 39.11f, 39.11f, 39.11f, 39.11f, 39.11f, 39.11f, 39.11f, 39.11f, 39.11f, 39.11f, 39.11f,
  43.11f, 43.11f, 43.11f, 43.11f, 43.11f, 43.11f, 43.11f, 43.11f, 43.11f, 43.11f, 43.11f, 43.11f, 43.11f, 43.11f, 43.11f, 43.11f,
  47.11f, 47.11f, 47.11f, 47.11f, 47.11f, 47.11f, 47.11f, 47.11f, 47.11f, 47.11f, 47.11f, 47.11f, 47.11f, 47.11f, 47.11f, 47.11f,
  51.11f, 51.11f, 51.11f, 51.11f, 51.11f, 51.11f, 51.11f, 51.11f, 51.11f, 51.11f, 51.11f, 51.11f, 51.11f, 51.11f, 51.11f, 51.11f,
  55.11f, 55.11f, 55.11f, 55.11f, 55.11f, 55.11f, 55.11f, 55.11f, 55.11f, 55.11f, 55.11f, 55.11f, 55.11f, 55.11f, 55.11f, 55.11f,
  59.11f, 59.11f, 59.11f, 59.11f, 59.11f, 59.11f, 59.11f, 59.11f, 59.11f, 59.11f, 59.11f, 59.11f, 59.11f, 59.11f, 59.11f, 59.11f,
  63.11f, 63.11f, 63.11f, 63.11f, 63.11f, 63.11f, 63.11f, 63.11f, 63.11f, 63.11f, 63.11f, 63.11f, 63.11f, 63.11f, 63.11f, 63.11f,
  67.11f, 67.11f, 67.11f, 67.11f, 67.11f, 67.11f, 67.11f, 67.11f, 67.11f, 67.11f, 67.11f, 67.11f, 67.11f, 67.11f, 67.11f, 67.11f,
  71.11f, 71.11f, 71.11f, 71.11f, 71.11f, 71.11f, 71.11f, 71.11f, 71.11f, 71.11f, 71.11f, 71.11f, 71.11f, 71.11f, 71.11f, 71.11
};


/*
  Table mapping inverse LED number to offset (Y+16*X) into scan plane.
  This gives a way to go from TLC index (order of shift-out) to position in
  scan plane (or led_angles_flash[] array).

  The first shifted-out outputs are B15, G15, R15, ... down to R0.
*/
static const uint16_t tlc_map_flash[SPIS][32] = {
  { 133, 101, 118, 86, 149, 117, 102, 70, 52, 20, 21, 85, 36, 53, 37, 69, 150, 135, 71, 103, 134, 151, 87, 119, 22, 38, 23, 55, 6, 54, 39, 7 },
  { 127, 94, 109, 95, 111, 110, 108, 78, 126, 157, 156, 124, 141, 125, 172, 140, 77, 92, 76, 44, 93, 61, 60, 75, 188, 220, 123, 155, 204, 91, 107, 139 },
  { 130, 114, 163, 131, 113, 146, 147, 115, 112, 97, 99, 65, 96, 81, 98, 80, 195, 84, 100, 132, 179, 211, 116, 148, 82, 50, 51, 68, 66, 83, 67, 35 },
  { 43, 58, 42, 74, 59, 27, 26, 90, 154, 122, 105, 73, 138, 106, 121, 89, 9, 57, 40, 8, 25, 41, 24, 56, 137, 152, 88, 120, 153, 136, 72, 104 },
  { 222, 207, 206, 191, 223, 221, 205, 190, 219, 218, 201, 200, 203, 202, 217, 216, 174, 158, 189, 142, 175, 159, 143, 173, 184, 168, 170, 171, 185, 169, 186, 187 },
  { 196, 197, 214, 215, 212, 213, 198, 199, 208, 210, 194, 177, 209, 192, 193, 176, 182, 166, 181, 180, 183, 167, 165, 164, 160, 144, 128, 162, 161, 145, 178, 129 }
};
static uint16_t tlc_map[SPIS][32];


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
  Adjust a scanplane buffer by right-shifting by 1 bit the first half.
  This is needed to handle the annoying extra one bit in the TLC5955 shift
  register that selects between GS or control latch.
  The first 25 32-bit words (of 49 total) are right-shifted. The first word
  is a dummy 0-word, the result is that we shift out 30 dummy bits, followed
  by one '0' bit to select GS for one TLC, then 768 data bits, then another
  '0' bit to select GS for the other TLC, and finally another 768 data bits.
*/
void
adjust_scanplane(uint32_t scanplane[25])
{
  /* We handle the 25 words in 5 iterations of 5 words each. */
  uint32_t counter = 5;

  __asm __volatile(
        "adds %[ptr], #100\n"   /* Also clears the carry flag. */
        "1:\n\t"
        "ldmdb %[ptr], {r2-r6}\n\t"
        "rrxs r6, r6\n\t"
        "rrxs r5, r5\n\t"
        "rrxs r4, r4\n\t"
        "rrxs r3, r3\n\t"
        "rrxs r2, r2\n\t"
        "stmdb %[ptr]!, {r2-r6}\n\t"
        "sub %[cnt], #1\n\t"
        "eors %[cnt], %[cnt], #0\n\t"
        "bne 1b\n"
        : [cnt] "+r" (counter),
          [ptr] "+r" (scanplane)
        :
        : "memory", "cc", "r2", "r3", "r4", "r5", "r6"
        );
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
                 uint32_t * __restrict b3,
                 uint32_t * __restrict b4,
                 uint32_t * __restrict b5,
                 uint32_t * __restrict b6)
{
  uint16_t *ps[6] = { (uint16_t *)b1, (uint16_t *)b2, (uint16_t *)b3,
                      (uint16_t *)b4, (uint16_t *)b5, (uint16_t *)b6 };
  uint8_t (*f)[LEDS_Y*LEDS_X*LEDS_TANG][3] = get_framebuf(1 - render_idx);
  uint32_t t, i;

#ifdef COUNTER_CLOCKWISE
  angle=(LEDS_TANG-1)-angle;
#endif
  for (t = 0; t < SPIS; ++t)
  {
    uint16_t *map = &tlc_map[t][0];
    uint16_t *angles = &led_angles[t][0];
    uint16_t *p = ps[t];
    *(uint32_t *)p = 0;
    p += 2;
    for (i = 0; i < 32; ++i)
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
    adjust_scanplane((uint32_t *)ps[t]);
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

    ToDo: Use __RBIT instead, and shift out little-endian.
  */
  for (i = 0; i < 256; ++i)
    gammas[i] = __REV16(gammas_flash[i]*16);

  for (j = 0; j < SPIS; ++j)
  {
    for (i = 0; i < 32; ++i)
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
  return led_center_distances_flash[y+x*LEDS_Y];
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
