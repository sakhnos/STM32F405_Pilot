/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "i2c.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "MAX3010x.h"
#include "ssd1306.h"
#include "fonts.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void I2CMasterTransmitDMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
//uint8_t readRegister8(uint8_t address, uint8_t reg);
//void writeRegister8(uint8_t address, uint8_t reg, uint8_t value);
//uint32_t readRegister8sh(uint8_t reg);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t dat[6]={3,4,5,6,7,8};
uint8_t txdat[5];
uint8_t ddat[127];
uint32_t dddat[16];
char buf[50]; 
bool _0x00 = false, _0x07 = false;
/* USER CODE END 0 */

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */
	
	HAL_Delay(1000);
	
	//I2CMasterTransmitDMA(&hi2c1, (uint16_t)MAX30105_ADDRESS<<1, (uint8_t*)dat, 5);
	SSD1306_Init();
	char str[] = "Hetrergfgffg";
	//SSD1306_Fill(1);
	SSD1306_Puts(str, &Font_7x10, 1);
	SSD1306_UpdateScreen();
	HAL_Delay(100);
	MAX30102Init();
	//I2CMasterTransmitDMA(&hi2c1, (uint16_t)MAX30105_ADDRESS<<1, (uint8_t*)dat, 5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_GPIO_WritePin(TEST_1_GPIO_Port, TEST_1_Pin, GPIO_PIN_SET);
		txdat[0]=0x00;
		HAL_I2C_Master_Transmit(&hi2c1, MAX30105_ADDRESS<<1, &txdat[0], 1, 100);
		
		HAL_I2C_Master_Receive(&hi2c1, MAX30105_ADDRESS<<1, &ddat[0], 1,100);
		txdat[0]=0x07;
		HAL_I2C_Master_Transmit(&hi2c1, MAX30105_ADDRESS<<1, &txdat[0], 1, 100);
		
		HAL_I2C_Master_Receive(&hi2c1, MAX30105_ADDRESS<<1, &ddat[0], 72,100);
		HAL_GPIO_WritePin(TEST_1_GPIO_Port, TEST_1_Pin, GPIO_PIN_RESET);
		//HAL_I2C_Master_Receive_DMA(&hi2c1, MAX30105_ADDRESS<<1, &ddat[0], 96);
		/*SSD1306_GotoXY(0, 0);
		SSD1306_Fill(SSD1306_COLOR_BLACK);
		sprintf(buf, "%d %d %d", ddat[0],ddat[1],ddat[2]);
		
		SSD1306_Puts(buf, &Font_7x10, 1);
		SSD1306_UpdateScreen();*/
		HAL_Delay(8);
		//HAL_GPIO_TogglePin(TEST_1_GPIO_Port, TEST_1_Pin);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*HAL_GPIO_WritePin(TEST_1_GPIO_Port, TEST_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TEST_2_GPIO_Port, TEST_2_Pin, GPIO_PIN_SET);
	txdat[0] = 0x00;
	HAL_I2C_Master_Transmit_IT(&hi2c1, MAX30105_ADDRESS<<1, &txdat[0], 1);
	//HAL_I2C_Master_Receive_DMA(&hi2c1, MAX30105_ADDRESS<<1, &ddat[0], 3);
	_0x00 = true;
	_0x07 = false;
	HAL_GPIO_WritePin(TEST_1_GPIO_Port, TEST_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TEST_2_GPIO_Port, TEST_2_Pin, GPIO_PIN_RESET);*/
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	/*if(_0x00 == true)
	{
		HAL_I2C_Master_Receive_IT(&hi2c1, MAX30105_ADDRESS<<1, &ddat[0], 1);
		//_0x00 = false;
	}
	else
//		while(
			HAL_I2C_Master_Receive_IT(&hi2c1, MAX30105_ADDRESS<<1, &ddat[0], 72);
//		!= HAL_OK)
		{
    
			if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
			{
				//Error_Handler();
			}   
		}*/
	//HAL_I2C_Master_Receive_IT(&hi2c1, MAX30105_ADDRESS<<1, &ddat[0], 127);
	
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	/*if(_0x00 == true)
	{
		_0x00 = false;
		txdat[0] = 0x07;
		HAL_I2C_Master_Transmit_IT(&hi2c1, MAX30105_ADDRESS<<1, &txdat[0], 1);
		_0x07 = true;
	}*/
	int i=0;
	for(i=0;i<16;i++)
	{
		dddat[i]=(ddat[3*i]<<16)|(ddat[3*i+1]<<8)|ddat[3*i+2];
	}
	//while(1);
}
void I2CMasterTransmitDMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
	 while(HAL_I2C_Master_Transmit_DMA(hi2c, DevAddress, pData, Size)!= HAL_OK)
  {
    /* Error_Handler() function is called when Timeout error occurs.
       When Acknowledge failure occurs (Slave don't acknowledge it's address)
       Master restarts communication */
    if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF)
    {
      Error_Handler();
    }
  }
}
/*uint8_t readRegister8(uint8_t address, uint8_t reg)
{
	uint8_t addr = reg;
  uint8_t data = 0;
  uint8_t d;
  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
  d = HAL_I2C_Master_Transmit(&hi2c1, MAX30105_ADDRESS<<1, &addr, 1, 1000);
  if ( d != HAL_OK) {
      return d;
  }

  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
  d = HAL_I2C_Master_Receive(&hi2c1, MAX30105_ADDRESS<<1, &data, 1, 1000);
  if ( d != HAL_OK) {
      return d;
	}
  return data;
}
void writeRegister8(uint8_t address, uint8_t reg, uint8_t value)
{
uint8_t addr = reg, data = value;

  uint8_t buf[] = {addr, data};
  uint8_t d;
  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
  d = HAL_I2C_Master_Transmit(&hi2c1, MAX30105_ADDRESS<<1, buf, 2, 1000);
}
uint32_t readRegister8sh(uint8_t reg)
{
  uint8_t pData[3];
  uint8_t d;
  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
  d = HAL_I2C_Master_Transmit(&hi2c1, MAX30105_ADDRESS<<1, &reg, 1, 1000);
  if ( d != HAL_OK) {
      return d;
  }

  while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
  d = HAL_I2C_Master_Receive(&hi2c1, MAX30105_ADDRESS<<1, &pData[0], 3, 1000);
  if ( d != HAL_OK) {
      return d;
	}
	return (((pData[0])<<16)|(pData[1]<<8)|(pData[2]));
}*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

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
