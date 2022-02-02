
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <pwm3901.h>
#include "main.h"
#include "hw_config.h"
#include "AnalisRS.h"
#include "variables.h"
#include "stm32f3xx_ll_iwdg.h"
#include "stm32f3xx_ll_tim.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_it.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
uint32_t CntPrnt=0,Cntpwdown=0,InputDetectStamp=0;
uint32_t ADNScnt=0;
uint16_t StageFlg=0,OneConfigYes=0,utmp16;
int8_t adxdy[2]={0,0};
uint8_t ad7[16];
uint8_t ad13[16];
float f1,f2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */


int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
//not used  EnableRTCBkpDomain();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
//used from timer in one pulse mode   MX_USART3_UART_Init();
  MX_TIM2_Init();
//  MX_CRC_Init();

//not used  MX_RTC_Init(); //?? EnableRTCBkpDomain(); //??again disable rtc protection
  MX_IWDG_Init();

  MState.StatusFlags|=PWM3901_Reset();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  MState.StatusFlags|=WF_ResetIfSet;
  OnDEPULSE_OnTrn_U3;
//  uint32_t LocalTimeCnt=SysTickCntr+25000;
//  while (LocalTimeCnt>SysTickCntr){LL_IWDG_ReloadCounter(IWDG);}
//  f1=SysTickCntr;
//  f2=f1/355.00f;
  RsThisSlaveDeviceN=1;
//  if ((Get_PA4_IDbit0)==0) RsThisSlaveDeviceN|=0x01;
//  if ((Get_PC13_IDbit1)==0) RsThisSlaveDeviceN|=0x02; //not valid for boards ver 3.0
  RSS.State=RSstRcv; //but here no still valid data received, then driver will make init
  RSS.CntTimeout=SysTickCntr;ADNScnt=SysTickCntr+ADNSSCANPER; //every 5msec
pak:
		LL_IWDG_ReloadCounter(IWDG);

		if (CntPrnt<=SysTickCntr)
		{
			StageFlg=1-StageFlg;
			CntPrnt=SysTickCntr+5000;
			if (StageFlg){OffCPUSecLED;}else{OnCPUSecLED;}
		}
/*
                if (TIM2->CNT>=32000000U) //timer counts with 72MHz, i.e. here we have 0.5s
                {
                        StageFlg=1-StageFlg;
                        TIM2->CNT=0;
                        if (StageFlg){OffCPUSecLED;}else{OnCPUSecLED;}
                }
*/

		if (RSS.ErrCod)
		{
			if (StageFlg) {if ((CntPrnt-4000)<=SysTickCntr) {OnCPUSecLED;}}
		}

#define REDMETHOD 1
/*
                if (ADNScnt<=SysTickCntr)
                {
                  ADNScnt=SysTickCntr+ADNSSCANPER; //every 50msec?
#if REDMETHOD==1
                  ReadMot(ad7);
                  if (ad7[0]&0x80)
                  {
                    MStateW.X+=((uint16_t)ad7[1]+((uint16_t)ad7[2]*256));MStateW.Y+=((uint16_t)ad7[3]+((uint16_t)ad7[4]*256));
                    MState.X=MStateW.X;MState.Y=MStateW.Y; uint32_t* pp7=(uint32_t*)ad7; pp7[0]=0;pp7[1]=0;
//                    *((volatile uint32_t*)&MState.X)=*((uint32_t*)&MStateW.X);
                    if (GetDEPULSE_OnTrn_U3) {OffDEPULSE_OnRcv_U3;}else{OnDEPULSE_OnTrn_U3;}
                  }
#endif
#if REDMETHOD==2
                  ReadBurstMotion(ad13,12); // dr0, obs1, x23 , y45, quality6, raw_sum7, raw_max8, raw_min9 , shutter_upper10, shutter_lower11
                  if  ( (ad13[0]&0x80) )//dr
                  {
                    MStateW.X+=((uint16_t)ad13[1]+((uint16_t)ad13[2]*256));MStateW.Y+=((uint16_t)ad13[3]+((uint16_t)ad13[4]*256));
                    MState.X=MStateW.X;MState.Y=MStateW.Y;uint32_t* pp13=(uint32_t*)ad13; pp13[0]=0;pp13[1]=0;pp13[2]=0;
                    if (GetDEPULSE_OnTrn_U3) {OffDEPULSE_OnRcv_U3;}else{OnDEPULSE_OnTrn_U3;}
                  }
#endif

                }
*/
		{
		static uint8_t newmov=0; //err check false MOV signal from PWM3901
		if (Get_MOV_EC4==0)
		{
			if (newmov){MState.StatusFlags|=0x10;goto pak;} //some error?
			newmov=1;
#if REDMETHOD==1
                  ReadMot(ad7);
                  if (ad7[0]&0x80)
                  {
                    MStateW.X+=((uint16_t)ad7[1]+((uint16_t)ad7[2]*256));MStateW.Y+=((uint16_t)ad7[3]+((uint16_t)ad7[4]*256));
                    MState.X=MStateW.X;MState.Y=MStateW.Y; uint32_t* pp7=(uint32_t*)ad7; pp7[0]=0;pp7[1]=0;
//                    *((volatile uint32_t*)&MState.X)=*((uint32_t*)&MStateW.X);
                    if (GetDEPULSE_OnTrn_U3) {OffDEPULSE_OnRcv_U3;}else{OnDEPULSE_OnTrn_U3;} //test led toggle
                  }
                  else
                  {
                	  MState.StatusFlags|=0x08; //some error?
                  }
#endif
#if REDMETHOD==2
                  ReadBurstMotion(ad13,12); // dr0, obs1, x23 , y45, quality6, raw_sum7, raw_max8, raw_min9 , shutter_upper10, shutter_lower11
                  if  ( (ad13[0]&0x80) )//dr
                  {
                    MStateW.X+=((uint16_t)ad13[1]+((uint16_t)ad13[2]*256));MStateW.Y+=((uint16_t)ad13[3]+((uint16_t)ad13[4]*256));
                    MState.X=MStateW.X;MState.Y=MStateW.Y;uint32_t* pp13=(uint32_t*)ad13; pp13[0]=0;pp13[1]=0;pp13[2]=0;
                    if (GetDEPULSE_OnTrn_U3) {OffDEPULSE_OnRcv_U3;}else{OnDEPULSE_OnTrn_U3;} //test led toggle
                  }
                  else
                  {
                	  MState.StatusFlags|=0x08; //some error?
                  }
#endif
		}
		else
		{
			newmov=0;
		}
		}
goto pak;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */



/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
