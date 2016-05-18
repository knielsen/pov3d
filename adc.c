#include "ledtorus.h"

#include <libopencm3/stm32/adc.h>


/*
  Reading the supply voltage to detect when the LiPo cell is getting
  drained to a dangerously low level.

  The voltage is on PC0, which is ADC channel 10 (on all of ADC1, ADC2, and
  ADC3). It is on a 10k ohm / 22 k ohm voltage divider, so the actual supply
  voltage is V = (22k+10k)/10k * V_measured.
*/

void
config_adc(void)
{
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_ADC1);

  adc_power_off(ADC1);

  gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
  gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO0);

  adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT|ADC_CCR_DMA_DISABLE|ADC_CCR_DELAY_5ADCCLK);
  adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);

  adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
  adc_disable_scan_mode(ADC1);
  adc_set_single_conversion_mode(ADC1);

  /*
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = 0;
  ADC_InitStructure.ADC_ExternalTrigConv = 0;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  */

  adc_enable_temperature_sensor();
  adc_power_on(ADC1);
}


uint32_t
adc_read(void)
{
  uint8_t channel = 10;

  adc_set_sample_time(ADC1, channel, ADC_SMPR_SMP_480CYC);
  adc_set_regular_sequence(ADC1, 1, &channel);
  adc_start_conversion_regular(ADC1);

  /* Wait for the ADC to complete. */
  while (!adc_eoc(ADC1));
    ;

  return adc_read_regular(ADC1);
}


float
voltage_read_vrefint_adjust(void)
{
  uint16_t adc_bat, adc_vrefint;
  uint8_t channel_bat = 10;
  uint8_t channel_vrefint = ADC_CHANNEL_VREF;

  /* Do an ADC measurement of the battery voltage. */
  adc_set_sample_time(ADC1, channel_bat, ADC_SMPR_SMP_480CYC);
  adc_set_regular_sequence(ADC1, 1, &channel_bat);
  adc_start_conversion_regular(ADC1);
  while (!adc_eoc(ADC1));
    ;
  adc_bat = adc_read_regular(ADC1);

  /* Do a corresponding measurement of the Vrefint internal 1.2V reference. */
  adc_set_sample_time(ADC1, channel_vrefint, ADC_SMPR_SMP_480CYC);
  adc_set_regular_sequence(ADC1, 1, &channel_vrefint);
  adc_start_conversion_regular(ADC1);
  while (!adc_eoc(ADC1));
    ;
  adc_vrefint = adc_read_regular(ADC1);

  /*
    The battery voltage is on a 32/10 voltage divider. The ADC reference
    voltage is the 3.3V VCC from the LDO.

    If the battery voltage drops low (<3.6V typical according to datasheet,
    depends on load, temperature, etc.), then the LDO may not be able to
    maintain output voltage and VCC (and hence ADC reference voltage) could
    drop below the expected 3.3V.

    By measuring the internal 1.2 V reference, we can detect this and adjust
    the measurement accordingly. Otherwise, we would overestimate the battery
    voltage as the LDO output drops below the nominal 3.3V.

    Vrefint = adc_vrefint/4095*Vref = 1.21V
    V_battery = adc_bat/4095*Vref*(32/10)

    So we can take Vref = 1.21V*4095/adc_vrefint, and hence corrected battery
    voltage as:

    V_bat = adc_bat/adc_vrefint*(32/10*1.21V)
  */
  return (float)adc_bat / (float)adc_vrefint * (32.0f/10.0f*1.21f);
}


float
voltage_read(void)
{
  uint32_t reading = adc_read();
  return ((float)reading)*((32.0f/10.0f)*3.3f/4095.0f);
}
