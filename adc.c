#include "ledtorus.h"

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
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  ADC_Cmd(ADC1, DISABLE);
  GPIO_StructInit(&GPIO_InitStructure);
  ADC_StructInit(&ADC_InitStructure);
  ADC_CommonStructInit(&ADC_CommonInitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = 0;
  ADC_InitStructure.ADC_ExternalTrigConv = 0;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_TempSensorVrefintCmd(ENABLE);
  ADC_Cmd(ADC1, ENABLE);
}


uint32_t
adc_read(void)
{
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);
  ADC_SoftwareStartConv(ADC1);

  /* Wait for the ADC to complete. */
  while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
    ;

  return ADC_GetConversionValue(ADC1);
}


float
voltage_read_vrefint_adjust(void)
{
  uint16_t adc_bat, adc_vrefint;

  /* Do an ADC measurement of the battery voltage. */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);
  ADC_SoftwareStartConv(ADC1);
  while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
    ;
  adc_bat = ADC_GetConversionValue(ADC1);

  /* Do a corresponding measurement of the Vrefint internal 1.2V reference. */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 1, ADC_SampleTime_480Cycles);
  ADC_SoftwareStartConv(ADC1);
  while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
    ;
  adc_vrefint = ADC_GetConversionValue(ADC1);

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
