/**
  ******************************************************************************
  * @file    stm32f4xx_conf.h  
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Library configuration file.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_CONF_H
#define __STM32F4xx_CONF_H

#if defined  (HSE_VALUE)
/* Redefine the HSE value; it's equal to 8 MHz on the STM32F4-DISCOVERY Kit */
 #undef HSE_VALUE
 #define HSE_VALUE    ((uint32_t)8000000) 
#endif /* HSE_VALUE */

/* Includes ------------------------------------------------------------------*/
/* Uncomment the line below to enable peripheral header file inclusion */
 #include "stm32f4xx_adc.h"
// #include "stm32f4xx_can.h"
// #include "stm32f4xx_crc.h"
// #include "stm32f4xx_cryp.h"
// #include "stm32f4xx_dac.h"
// #include "stm32f4xx_dbgmcu.h"
// #include "stm32f4xx_dcmi.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_exti.h"
// #include "stm32f4xx_flash.h"
// #include "stm32f4xx_fsmc.h"
// #include "stm32f4xx_hash.h"
#include "stm32f4xx_gpio.h"
// #include "stm32f4xx_i2c.h"
// #include "stm32f4xx_iwdg.h"
// #include "stm32f4xx_pwr.h"
#include "stm32f4xx_rcc.h"
// #include "stm32f4xx_rng.h"
// #include "stm32f4xx_rtc.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
// #include "stm32f4xx_wwdg.h"
#include "misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* If an external clock source is used, then the value of the following define 
   should be set to the value of the external clock source, else, if no external 
   clock is used, keep this define commented */
/*#define I2S_EXTERNAL_CLOCK_VAL   12288000 */ /* Value of the external clock in Hz */


/* Configuration for SDIO driver. */

/* Card detect pin on PA15. */
#define SD_DETECT_PIN                    GPIO_Pin_15
#define SD_DETECT_GPIO_PORT              GPIOA
#define SD_DETECT_GPIO_CLK               RCC_AHB1Periph_GPIOA

//#define SDIO_FIFO_ADDRESS                ((uint32_t)0x40012C80)
#define SDIO_FIFO_ADDRESS                ((uint32_t)&SDIO->FIFO)
/**
  * @brief  SDIO Intialization Frequency (400KHz max)
  * Obtained from 48 MHz PLL output, result is 48MHz/(SDIO_INIT_CLK_DIV+2)
  */
#define SDIO_INIT_CLK_DIV                ((uint8_t)0x76) /* 400kHz */
/**
  * @brief  SDIO Data Transfer Frequency (25MHz max)
  * Obtained from 48 MHz PLL output, result is 48MHz/(SDIO_TRANSFER_CLK_DIV+2)
  */
#define SDIO_TRANSFER_CLK_DIV            ((uint8_t)0) /* 24 MHz */

#define SD_SDIO_DMA                   DMA2
#define SD_SDIO_DMA_CLK               RCC_AHB1Periph_DMA2

//#define SD_SDIO_DMA_STREAM3	          3
#define SD_SDIO_DMA_STREAM6           6

#ifdef SD_SDIO_DMA_STREAM3
 #define SD_SDIO_DMA_STREAM            DMA2_Stream3
 #define SD_SDIO_DMA_CHANNEL           DMA_Channel_4
 #define SD_SDIO_DMA_FLAG_FEIF         DMA_FLAG_FEIF3
 #define SD_SDIO_DMA_FLAG_DMEIF        DMA_FLAG_DMEIF3
 #define SD_SDIO_DMA_FLAG_TEIF         DMA_FLAG_TEIF3
 #define SD_SDIO_DMA_FLAG_HTIF         DMA_FLAG_HTIF3
 #define SD_SDIO_DMA_FLAG_TCIF         DMA_FLAG_TCIF3
 #define SD_SDIO_DMA_IRQn              DMA2_Stream3_IRQn
 #define SD_SDIO_DMA_IRQHANDLER        DMA2_Stream3_IRQHandler
 #define SD_SDIO_DMA_ISR               (DMA2->LISR)
#elif defined SD_SDIO_DMA_STREAM6
 #define SD_SDIO_DMA_STREAM            DMA2_Stream6
 #define SD_SDIO_DMA_CHANNEL           DMA_Channel_4
 #define SD_SDIO_DMA_FLAG_FEIF         DMA_FLAG_FEIF6
 #define SD_SDIO_DMA_FLAG_DMEIF        DMA_FLAG_DMEIF6
 #define SD_SDIO_DMA_FLAG_TEIF         DMA_FLAG_TEIF6
 #define SD_SDIO_DMA_FLAG_HTIF         DMA_FLAG_HTIF6
 #define SD_SDIO_DMA_FLAG_TCIF         DMA_FLAG_TCIF6
 #define SD_SDIO_DMA_IRQn              DMA2_Stream6_IRQn
 #define SD_SDIO_DMA_IRQHANDLER        DMA2_Stream6_IRQHandler
 #define SD_SDIO_DMA_ISR               (DMA2->HISR)
#endif /* SD_SDIO_DMA_STREAM3 */


void SD_LowLevel_DeInit(void);
void SD_LowLevel_Init(void);
void SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize);
void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize);

/* End of SDIO driver configuration. */


/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *   which reports the name of the source file and the source
  *   line number of the call that failed. 
  *   If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F4xx_CONF_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
