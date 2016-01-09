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
#include "i2c.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */
uint16_t gyrodata[3];
uint16_t acceldata[3];
uint8_t dat;
//uint16_t magnetodata[6];
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


uint8_t sendat;
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  HAL_StatusTypeDef status;
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C3_Init();

  uint8_t data[12];
  status=HAL_I2C_Mem_Read(&hi2c3,0xD1,0x75,1,&dat,1,10000);
  /* USER CODE END 2 */
  sendat=0x00;
  HAL_I2C_Mem_Write(&hi2c3,0xD0,0x6B,1,&sendat,1,10000);
  sendat=0x10;
  HAL_I2C_Mem_Write(&hi2c3,0xD0,0x1C,1,&sendat,1,10000);
  sendat=0x18;
  HAL_I2C_Mem_Write(&hi2c3,0xD0,0x18,1,&sendat,1,10000);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
   status=HAL_I2C_Mem_Read(&hi2c3,0xD1,0x3B,1,&dat,1,10000);
   *acceldata=(dat)<<8;
   status=HAL_I2C_Mem_Read(&hi2c3,0xD1,0x3C,1,&dat,1,1000);
   *acceldata= *acceldata | dat;
   HAL_Delay(1);
   //*acceldata=*acceldata << 16;
   status=HAL_I2C_Mem_Read(&hi2c3,0xD1,0x3D,1,&dat,1,10000);
   acceldata[1]=(dat)<<8;
   status=HAL_I2C_Mem_Read(&hi2c3,0xD1,0x3E,1,&dat,1,1000);
   acceldata[1]= acceldata[1] | dat;
   HAL_Delay(1);
   //*acceldata=*acceldata << 16;
   status=HAL_I2C_Mem_Read(&hi2c3,0xD1,0x3F,1,&dat,1,10000);
   acceldata[2]=(dat)<<8;
   status=HAL_I2C_Mem_Read(&hi2c3,0xD1,0x40,1,&dat,1,1000);
   acceldata[2]= acceldata[2] | dat;
   HAL_Delay(1);


    status=HAL_I2C_Mem_Read(&hi2c3,0xD1,0x43,1,&dat,1,10000);
   gyrodata[0]=(dat)<<8;
   status=HAL_I2C_Mem_Read(&hi2c3,0xD1,0x44,1,&dat,1,1000);
   *gyrodata= *gyrodata | dat;
   HAL_Delay(1);
   //*gyrodata = *gyrodata << 16;
     status=HAL_I2C_Mem_Read(&hi2c3,0xD1,0x45,1,&dat,1,10000);
   gyrodata[1]=(dat)<<8;
   status=HAL_I2C_Mem_Read(&hi2c3,0xD1,0x46,1,&dat,1,1000);
   gyrodata[1]= gyrodata[1] | dat;
   HAL_Delay(1);
   //*gyrodata = *gyrodata << 16;
     status=HAL_I2C_Mem_Read(&hi2c3,0xD1,0x47,1,&dat,1,10000);
   gyrodata[2]=(dat)<<8;
   status=HAL_I2C_Mem_Read(&hi2c3,0xD1,0x48,1,&dat,1,1000);
   gyrodata[2]= gyrodata[2] | dat;
   HAL_Delay(1);
  /* 
  /*    HAL_I2C_Master_Receive_IT(&hi2c3,0x3B,&dat,1);
    *acceldata = (dat)<<8;
    HAL_I2C_Master_Receive_IT(&hi2c3,0x3C,&dat,1);
    *acceldata = *acceldata | dat;
    HAL_I2C_Master_Receive_IT(&hi2c3,0x43,&dat,1);
    *gyrodata = (dat)<<8;
    HAL_I2C_Master_Receive_IT(&hi2c3,0x43,&dat,1);
    *gyrodata = *gyrodata | dat;
  /* USER CODE BEGIN 3 */

 
 //HAL_I2C_Master_Transmit(&hi2c3, 0x18, (uint8_t *)data,1,1000); 
  //status=HAL_I2C_Master_Receive(&hi2c3,0xD1,&dat,1,1000);
  
 HAL_Delay(1);
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

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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
