/**
  ******************************************************************************
  * @file    hw_config.h
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    11-July-2011
  * @brief   Target config file module.
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
#ifndef __HARDWARE_CONFIG_H
#define __HARDWARE_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx.h"

#include "stm32f3xx_ll_adc.h"
#include "stm32f3xx_ll_crc.h"
#include "stm32f3xx_ll_iwdg.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_cortex.h"
#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_rtc.h"
#include "stm32f3xx_ll_tim.h"
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_gpio.h"

//#include "board.h"
#define CurrentSVpBV (uint16_t)(0x2003)

//#include "variables.h"
//#include "AnalisRS.h"

#define OPPSysTick_IRQn             0x06
//#define xOPPRTC_IRQn                 0x0c  //not exist here
//#define OPPOTG_FS_IRQn               0x07
//#define OPPOTG_HS_IRQn               0x08
//#define OPPUSART4_IRQn             0x05
#define OPPUSART1_IRQn             0x03
#define OPPUSART2_IRQn             0x04
#define OPPUSART3_IRQn               0x05
#define OPPTIM16_IRQn				0x02 //highest
//#define OPPTIM7_IRQn               0x0a
//#define OPPEXTI9_5_IRQn           0x0c

/* Exported macros -----------------------------------------------------------*/

#define OnCPUSecLED LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_14) //swclk
#define OffCPUSecLED LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_14) //swclk

#define OffDE_OnRcv_U1 LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_12)
#define OnDE_OnTrn_U1  LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_12)

#define OffDE_OnRcv_U2 LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_1)
#define OnDE_OnTrn_U2  LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_1)

#define OffDEPULSE_OnRcv_U3 LL_GPIO_ResetOutputPin(GPIOF,LL_GPIO_PIN_6)
#define OnDEPULSE_OnTrn_U3  LL_GPIO_SetOutputPin(GPIOF,LL_GPIO_PIN_6)
#define GetDEPULSE_OnTrn_U3  (GPIOF->ODR&LL_GPIO_PIN_6)
#define Offu3SIGN LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_5)
#define Onu3SIGN  LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_5)


#define Get_PE8_A28P (GPIOE->IDR&LL_GPIO_PIN_8)
#define Get_PE9_A28M (GPIOE->IDR&LL_GPIO_PIN_9)
#define Get_PB0_A16P (GPIOB->IDR&LL_GPIO_PIN_0)
#define Get_PB1_A16M (GPIOB->IDR&LL_GPIO_PIN_1)
#define Get_PB14_A38P (GPIOB->IDR&LL_GPIO_PIN_14)
#define Get_PB15_A38M (GPIOB->IDR&LL_GPIO_PIN_15)

#define Get_PA4_IDbit0 (GPIOA->IDR&LL_GPIO_PIN_4)
#define Get_PC13_IDbit1 (GPIOC->IDR&LL_GPIO_PIN_13) //not valid for boards 3.0

#define Off_PB2_EC1 LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_2)
#define On_PB2_EC1  LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_2)
#define Lo_NCS_EC1 LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_2)
#define Hi_NCS_EC1  LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_2)

//#define Off_PB4_EC2 LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_4)
//#define On_PB4_EC2  LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_4)
//#define Lo_MISO_EC2 LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_4)
//#define Hi_MISO_EC2  LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_4)
#define Get_MISO_EC2 (GPIOB->IDR&LL_GPIO_PIN_4)


#define Off_PF7_EC5 LL_GPIO_ResetOutputPin(GPIOF,LL_GPIO_PIN_7)
#define On_PF7_EC5  LL_GPIO_SetOutputPin(GPIOF,LL_GPIO_PIN_7)
#define Lo_MOSI_EC5 LL_GPIO_ResetOutputPin(GPIOF,LL_GPIO_PIN_7)
#define Hi_MOSI_EC5  LL_GPIO_SetOutputPin(GPIOF,LL_GPIO_PIN_7)

#define Off_PA8_EC6 LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_8)
#define On_PA8_EC6  LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_8)
#define Lo_SCLK_EC6 LL_GPIO_ResetOutputPin(GPIOA,LL_GPIO_PIN_8)
#define Hi_SCLK_EC6  LL_GPIO_SetOutputPin(GPIOA,LL_GPIO_PIN_8)

#define Off_PB6_EC3 LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_6)
#define On_PB6_EC3  LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_6)
#define Lo_RST_EC3 LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_6)
#define Hi_RST_EC3  LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_6)

//#define Off_PB8_EC4 LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_8)
//#define On_PB8_EC4  LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_8)
//#define Lo_NPD_EC4 LL_GPIO_ResetOutputPin(GPIOB,LL_GPIO_PIN_8)
//#define Hi_NPD_EC4  LL_GPIO_SetOutputPin(GPIOB,LL_GPIO_PIN_8)
#define Get_MOV_EC4 (GPIOB->IDR&LL_GPIO_PIN_8)


/* Exported functions ------------------------------------------------------- */
 void LL_Init(void);
 void SystemClock_Config(void);

 void MX_GPIO_Init(void);
 void MX_USART1_UART_Init(void);
 void MX_USART2_UART_Init(void);
 //void MX_USART3_UART_Init(void);
 void MX_IWDG_Init(void);
 void MX_TIM2_Init(void);
 void MX_CRC_Init(void);

 void EnableRTCBkpDomain(void);

  void _Error_Handler(char *, int);

  #define Error_Handler() _Error_Handler(__FILE__, __LINE__)


#ifdef __cplusplus
}
#endif

#endif /* __HARDWARE_CONFIG_H */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
