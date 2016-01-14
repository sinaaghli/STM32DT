/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f2xx_hal.h"
//#include "tim.h"
#include "gpio.h"


void SystemClock_Config(void);

#define sens_gpio     GPIOD
#define sens_ph1_pin  GPIO_PIN_11
#define sens_ph2_pin  GPIO_PIN_10
#define sens_ph3_pin  GPIO_PIN_9
#define EnGate_gpio   GPIOB
#define EnGate_pin    GPIO_PIN_8
#define DcCal_gpio    GPIOD
#define DcCal_pin     GPIO_PIN_7
#define Led_gpio      GPIOE
#define Led_pin       GPIO_PIN_2

#define Phase_gpio    GPIOB
#define ph_u_h        GPIO_PIN_0
#define ph_u_l        GPIO_PIN_14
#define ph_v_h        GPIO_PIN_1
#define ph_v_l        GPIO_PIN_5
#define ph_w_h        GPIO_PIN_7
#define ph_w_l        GPIO_PIN_6

void ApplyPhase(char sens) {
  switch(sens) {
    case 5:   
      HAL_GPIO_WritePin(Phase_gpio,ph_u_h,SET);
      HAL_GPIO_WritePin(GPIOE,ph_u_l,RESET);
      HAL_GPIO_WritePin(Phase_gpio,ph_v_h,RESET);
      HAL_GPIO_WritePin(Phase_gpio,ph_v_l,SET);
      HAL_GPIO_WritePin(Phase_gpio,ph_w_h,RESET);
      HAL_GPIO_WritePin(Phase_gpio,ph_w_l,RESET);
      break;
    case 4:   
      HAL_GPIO_WritePin(Phase_gpio,ph_u_h,SET);
      HAL_GPIO_WritePin(GPIOE,ph_u_l,RESET);
      HAL_GPIO_WritePin(Phase_gpio,ph_v_h,RESET);
      HAL_GPIO_WritePin(Phase_gpio,ph_v_l,RESET);
      HAL_GPIO_WritePin(Phase_gpio,ph_w_h,RESET);
      HAL_GPIO_WritePin(Phase_gpio,ph_w_l,SET);
      break;
    case 6:   
      HAL_GPIO_WritePin(Phase_gpio,ph_u_h,RESET);
      HAL_GPIO_WritePin(GPIOE,ph_u_l,RESET);
      HAL_GPIO_WritePin(Phase_gpio,ph_v_h,SET);
      HAL_GPIO_WritePin(Phase_gpio,ph_v_l,RESET);
      HAL_GPIO_WritePin(Phase_gpio,ph_w_h,RESET);
      HAL_GPIO_WritePin(Phase_gpio,ph_w_l,SET);
      break;
    case 2:   
      HAL_GPIO_WritePin(Phase_gpio,ph_u_h,RESET);
      HAL_GPIO_WritePin(GPIOE,ph_u_l,SET);
      HAL_GPIO_WritePin(Phase_gpio,ph_v_h,SET);
      HAL_GPIO_WritePin(Phase_gpio,ph_v_l,RESET);
      HAL_GPIO_WritePin(Phase_gpio,ph_w_h,RESET);
      HAL_GPIO_WritePin(Phase_gpio,ph_w_l,RESET);
      break;
    case 3:   
      HAL_GPIO_WritePin(Phase_gpio,ph_u_h,RESET);
      HAL_GPIO_WritePin(GPIOE,ph_u_l,SET);
      HAL_GPIO_WritePin(Phase_gpio,ph_v_h,RESET);
      HAL_GPIO_WritePin(Phase_gpio,ph_v_l,RESET);
      HAL_GPIO_WritePin(Phase_gpio,ph_w_h,SET);
      HAL_GPIO_WritePin(Phase_gpio,ph_w_l,RESET);
      break;
    case 1: 
      HAL_GPIO_WritePin(Phase_gpio,ph_u_h,RESET);
      HAL_GPIO_WritePin(GPIOE,ph_u_l,RESET);
      HAL_GPIO_WritePin(Phase_gpio,ph_v_h,RESET);
      HAL_GPIO_WritePin(Phase_gpio,ph_v_l,SET);
      HAL_GPIO_WritePin(Phase_gpio,ph_w_h,SET);
      HAL_GPIO_WritePin(Phase_gpio,ph_w_l,RESET);
      break;
    default:
      while(1);
    }
}

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
//  MX_TIM3_Init();
//  MX_TIM4_Init();
//
//  HAL_TIM_Base_Start(&htim3);
//  HAL_TIM_Base_Start(&htim4);
//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
//  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_GPIO_WritePin(Led_gpio,Led_pin,SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(Led_gpio,Led_pin,RESET);

  HAL_GPIO_WritePin(DcCal_gpio,DcCal_pin,RESET);
  HAL_GPIO_WritePin(EnGate_gpio,EnGate_pin,SET);

  char state;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    state = HAL_GPIO_ReadPin(sens_gpio,sens_ph1_pin);
    state = state << 1;
    state |= HAL_GPIO_ReadPin(sens_gpio,sens_ph2_pin);
    state = state << 1;
    state |= HAL_GPIO_ReadPin(sens_gpio,sens_ph3_pin);

    ApplyPhase(state);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
