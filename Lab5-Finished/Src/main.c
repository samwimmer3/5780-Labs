/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

	//Enable GPIO B and C 
  RCC-> AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC-> AHBENR |= RCC_AHBENR_GPIOCEN;
	
	
	/////////LED
	//set pins to general purpose output mode in the moder register
	GPIOC->MODER |= (1<<14) | (1<<12);
	GPIOC->MODER &= ~(1<<15);
	GPIOC->MODER &= ~(1<<13);
	GPIOC->MODER |= (1<<18) | (1<<16);
	GPIOC->MODER &= ~(1<<19);
	GPIOC->MODER &= ~(1<<17);
	
	//set to push pull output in OTYPER reg
	GPIOC->OTYPER &= ~(1<<12);
	GPIOC->OTYPER &= ~(1<<13);
	GPIOC->OTYPER &= ~(1<<14);
	GPIOC->OTYPER &= ~(1<<15);
	GPIOC->OTYPER &= ~(1<<8);
	GPIOC->OTYPER &= ~(1<<9);
	
	//set pins low speed in OSPEEDR reg
	GPIOC->OSPEEDR &= ~(1<<12);
	GPIOC->OSPEEDR &= ~(1<<14);
	GPIOC->OSPEEDR &= ~(1<<18);
	GPIOC->OSPEEDR &= ~(1<<16);

	//set to no pullup/down resistors in PUPDR reg
	GPIOC->PUPDR &= ~(1<<12);
	GPIOC->PUPDR &= ~(1<<13);
	GPIOC->PUPDR &= ~(1<<14);
	GPIOC->PUPDR &= ~(1<<15);
	GPIOC->PUPDR &= ~(1<<16);
	GPIOC->PUPDR &= ~(1<<17);
	GPIOC->PUPDR &= ~(1<<18);
	GPIOC->PUPDR &= ~(1<<19);
	
	//One high and one low
	GPIOC->ODR &= ~(1<<6);
	GPIOC->ODR &= ~(1<<7);
	GPIOC->ODR &= ~(1<<8);
	GPIOC->ODR &= ~(1<<9);
	//////////////////////////////////
	
	
	//PB11 to AF, open drain, and I2C2_SDA for AF
	GPIOB -> MODER |= (1<<23);
	GPIOB -> MODER &= ~(1<<22);
	
	GPIOB -> OTYPER |= (1<<11);
	
	GPIOB -> AFR[1] &= ~(1<<15);
	GPIOB -> AFR[1] &= ~(1<<14);
	GPIOB -> AFR[1] &= ~(1<<13);
	GPIOB -> AFR[1] |= (1<<12);
	//-------------------------------------------
	
	
	
	//PB13 to AF, open drain, and I2C2_SCL for AF
	GPIOB -> MODER |= (1<<27);
	GPIOB -> MODER &= ~(1<<26);
	
	GPIOB -> OTYPER |= (1<<13);
	
	GPIOB -> AFR[1] &= ~(1<<23);
	GPIOB -> AFR[1] |= (1<<22);
	GPIOB -> AFR[1] &= ~(1<<21);
	GPIOB -> AFR[1] |= (1<<20);
	//-------------------------------------------	
	
	
	//PB14 to output, push pull and initialize high
	GPIOB -> MODER &= ~(1<<29);
	GPIOB -> MODER |=  (1<<28);
	
	GPIOB -> OTYPER &= ~(1<<14);
	
	GPIOC->ODR |= (1<<14);
	//-------------------------------------------		
	
	
	//PC0 to output, push pull and initialize high
	GPIOC -> MODER |= (1<<1);
	GPIOC -> MODER &= ~(1<<0);
	
	GPIOB -> OTYPER |= (1<<0);
	
	GPIOC->ODR |= (1<<0);
	//-------------------------------------------		
	
	
	//Enable I2C2
	RCC -> APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	
	///////////// TIMINGR
	//PRESC == 1
	I2C2 -> TIMINGR |= (1<<28);
	I2C2 -> TIMINGR &= ~(1<<29);
	I2C2 -> TIMINGR &= ~(1<<30);
	I2C2 -> TIMINGR &= ~(1<<31);
	
	//SCLL == 0x13 = 00010011
	I2C2 -> TIMINGR &= ~(1<<7);
	I2C2 -> TIMINGR &= ~(1<<6);
	I2C2 -> TIMINGR &= ~(1<<5);
	I2C2 -> TIMINGR |= (1<<4);
	I2C2 -> TIMINGR &= ~(1<<3);
	I2C2 -> TIMINGR &= ~(1<<2);
	I2C2 -> TIMINGR |= (1<<1);
	I2C2 -> TIMINGR |= (1<<0);
	
	//SCLH == 0xF = 00001111
	I2C2 -> TIMINGR &= ~(1<<15);
	I2C2 -> TIMINGR &= ~(1<<14);
	I2C2 -> TIMINGR &= ~(1<<13);
	I2C2 -> TIMINGR &= ~(1<<12);
	I2C2 -> TIMINGR |= (1<<11);
	I2C2 -> TIMINGR |= (1<<10);
	I2C2 -> TIMINGR |= (1<<9);
	I2C2 -> TIMINGR |= (1<<8);
	
	//SDADEL == 0x2 = 0010
	I2C2 -> TIMINGR &= ~(1<<19);
	I2C2 -> TIMINGR &= ~(1<<18);
	I2C2 -> TIMINGR |= (1<<17);
	I2C2 -> TIMINGR &= ~(1<<16);
	
	//SCLDEL == 0x4 = 0100
	I2C2 -> TIMINGR &= ~(1<<23);
	I2C2 -> TIMINGR |= (1<<22);
	I2C2 -> TIMINGR &= ~(1<<21);
	I2C2 -> TIMINGR &= ~(1<<20);
	//////////////////////////////
	
	//Enable PE in CR1
	I2C2 -> CR1 |= (1<<0);
	
	
	
	//Set Transaction parameters in CR2
	
	//Set slave address == 0x6b = 01101011
	I2C2 -> CR2 |= (0x6b<<1);
	
	//1 byte
	I2C2 -> CR2 &= ~(1<<23);
	I2C2 -> CR2 &= ~(1<<22);
	I2C2 -> CR2 &= ~(1<<21);
	I2C2 -> CR2 &= ~(1<<20);
	I2C2 -> CR2 &= ~(1<<19);
	I2C2 -> CR2 &= ~(1<<18);
	I2C2 -> CR2 &= ~(1<<17);
	I2C2 -> CR2 |= (1<<16);
	
	//RD_WRN to write = 0
	I2C2 -> CR2 &= ~(1<<10);
	
	//Set Start
	I2C2 -> CR2 |= (1<<13);
	
	int red = 6;
	int blue = 7;
	int green = 9;
	int orange = 8;	
	
	//wait for txis or nackf
	while((I2C2 ->ISR & (I2C_ISR_NACKF |I2C_ISR_TXIS)) == 0)
	{
	}
	
	if (I2C_ISR_NACKF!=0) GPIOC->ODR ^= (1<<blue);
	
	//address of who am i reg
	I2C2->TXDR |= (0x0F<<0);
	
	//transfer complete wait
	while(I2C_ISR_TC == 0) 
		{
		}
		
		
	//Set Transaction parameters in CR2
	
	//Set slave address == 0x6b = 01101011
	I2C2 -> CR2 |= (0x6b<<1);
	
	//1 byte
	I2C2 -> CR2 &= ~(1<<23);
	I2C2 -> CR2 &= ~(1<<22);
	I2C2 -> CR2 &= ~(1<<21);
	I2C2 -> CR2 &= ~(1<<20);
	I2C2 -> CR2 &= ~(1<<19);
	I2C2 -> CR2 &= ~(1<<18);
	I2C2 -> CR2 &= ~(1<<17);
	I2C2 -> CR2 |= (1<<16);
	
	//RD_WRN to read = 1
	I2C2 -> CR2 |= (1<<10);
	
	//Set Start
	I2C2 -> CR2 |= (1<<13);
	
	//wait for txis or nackf
	while((I2C2 ->ISR & (I2C_ISR_NACKF |I2C_ISR_RXNE)) == 0)
	{
	}
	
	if (I2C_ISR_NACKF!=0) GPIOC->ODR ^= (1<<orange);
	
	//transfer complete wait
	while(I2C_ISR_TC == 0) 
		{
		}
		
	if(I2C_RXDR_RXDATA == 0xD4)
		GPIOC->ODR ^= (1<<green);
	else
		GPIOC->ODR ^= (1<<red);
		
	//Set Stop
	I2C2 -> CR2 |= (1<<14);
  
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
