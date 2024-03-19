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



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  
  /* Configure the system clock */
  SystemClock_Config();

	RCC->AHBENR   |= RCC_AHBENR_GPIOAEN ;
  	RCC->AHBENR   |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR   |= RCC_AHBENR_GPIOCEN;
	RCC->APB2ENR  |= RCC_APB2ENR_ADC1EN;
	RCC-> APB1ENR |= RCC_APB1ENR_DACEN ;
	

	/////////LED/////////////////////////////////////////
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
	
	int red = 6;
	int green = 9;
	int blue = 7;
	int orange = 8;
	////////////////////////////////////////////////////
	
	
	
	
	////////ADC///////////////////////////////////////////
	//ADC_IN14 = PC4
	GPIOC->MODER |= (1<<9) | (1<<8); //analog mode
	GPIOC->OTYPER |= (1<<4); //open drain
	
	//Configure the ADC to 8-bit resolution, 
	//continuous conversion mode, 
	//hardware triggers disabled (software trigger only).
	ADC1->CFGR1 |= (1<<4);
	ADC1->CFGR1 |= (1<<13);
	ADC1->CFGR1 &= ~(1<<11);
	ADC1->CFGR1 &= ~(1<<10);
	
	ADC1->CHSELR |= (1<<14); //chnl 14 selected for conversion
	
	//Perform a self-calibration
	//Ensure that ADEN = 0 and DMAEN = 0.
	//2. Set ADCAL = 1.
	//3. Wait until ADCAL = 0.
		if ((ADC1->CR & ADC_CR_ADEN) != 0) 
	{
	ADC1->CR |= ADC_CR_ADDIS; 
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0)
	{
	}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
	ADC1->CR |= ADC_CR_ADCAL; 
	while ((ADC1->CR & ADC_CR_ADCAL) != 0)
	{
	}
	//////////////
	//Enable the adc
	ADC1->ISR |= (1<<0); //adry to 1
	ADC1->CR |= (1<<0); //aden to 1
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0)
	{
	 ADC1->CR |= (1<<0); //continue aden to 1
	}
	//////////////
	//Start ADC
	ADC1->CR |= (1<<2);
	////////////////////////////////////////////////////////

	
	
	////////////DAC/////////////////////////////////////////
	//PA4 for DAC_OUT1
	GPIOA->MODER |= (1<<9) | (1<<8);
	GPIOA->OTYPER |= (1<<4); //open drain
	
	//Set Channel to software trigger
	DAC1->CR |= (0x7<<3);
	
	//Enable channel
	DAC1->CR |= (1<<0);
	
	// Triangle Wave: 8-bit, 32 samples/cycle
	const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,
	190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};
	
	int index = 0;
	////////////////////////////////////////////////////////
	
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
		//////////PART 1///////////////////////
		int pwr = ADC1->DR;
		
		if(pwr > 10) GPIOC->ODR |= (1<<blue); 
		else  GPIOC->ODR &= ~(1<<blue);
		
		if(pwr > 100) GPIOC->ODR |= (1<<orange); 
		else  GPIOC->ODR &= ~(1<<orange);
		
		if(pwr > 150) GPIOC->ODR |= (1<<red); 
		else  GPIOC->ODR &= ~(1<<red);
		
		if(pwr > 250) GPIOC->ODR |= (1<<green); 
		else  GPIOC->ODR &= ~(1<<green);
		///////////////////////////////////////
		
		
		/////////Part 2///////////////////////
		DAC1->DHR8R1 = triangle_table[index];
		index++;
		if(index >= 32) index = 0;
		HAL_Delay(1);
		//////////////////////////////////////
		
  }
  /* USER CODE END 3 */
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
