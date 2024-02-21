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

void SystemClock_Config(void);

//Once a character is typed it will transmit the given char
void transmitChar(char x)
{
	//Check to see if empty
	while(1)
	{
		if((USART3->ISR & (1<<7)) == (1<<7))
		{
			break;
		}
	}
		//Transmit Character
		USART3 -> TDR = x;
}

//Loops through the char array and transmits every character
void transmitString(char* x)
{	
	int pos = 0;
	
	while(x[pos] != NULL)
	{
	//Check to see if empty
	while(1)
	{
		if((USART3->ISR & (1<<7)) == (1<<7))
		{
			break;
		}
	}
		//Transmit Character
		USART3 -> TDR = x[pos];
		pos++;
	}
}

//Checks the recieved register to make sure a char is entered and returns it
char recievedChar(void)
{
	while(!(USART3->ISR & USART_ISR_RXNE))
	{
	}
	
	return(char)(USART3 -> RDR & 0xFF);
	
}



volatile char input; //input from keyboard
volatile int colorFlag; //flag for a color being input
volatile int actionFlag; //flag for a action number being input

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	HAL_Init();
	SystemClock_Config();
	
	//USING PIN     and     PB11 = USART RX
	
	//Enable Usart 3
	RCC-> APB1ENR = RCC_APB1ENR_USART3EN;
	
	//Enable pins and set alternative function mode
  	RCC-> AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOB->MODER |= (1<<23) | (1<<21);
	GPIOB->MODER &= ~(1<<22);
	GPIOB->MODER &= ~(1<<20);
	
	//Set Alternate function 4
	GPIOB -> AFR[1] &= ~(1<<15);
	GPIOB -> AFR[1] |= (1<<14);
	GPIOB -> AFR[1] &= ~(1<<13);
	GPIOB -> AFR[1] &= ~(1<<12);
	
	GPIOB -> AFR[1] &= ~(1<<11);
	GPIOB -> AFR[1] |= (1<<10);
	GPIOB -> AFR[1] &= ~(1<<9);
	GPIOB -> AFR[1] &= ~(1<<8);
	
	//Set Baud Rate
	USART3->BRR = HAL_RCC_GetHCLKFreq()/115200;
	
	//Enable Transmitter and reciever
	USART3 -> CR1 |= (1<<5);
	USART3 -> CR1 |= (1<<3);
	USART3 -> CR1 |= (1<<2);
	USART3 -> CR1 |= (1<<0);
	
	
	
	//Initialize LEDs
	RCC -> AHBENR |= RCC_AHBENR_GPIOCEN; //enable peripheral clock
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
	
	//ALL LEDS OFF
	GPIOC->ODR &= ~(1<<6);
	GPIOC->ODR &= ~(1<<7);
	GPIOC->ODR &= ~(1<<8);
	GPIOC->ODR &= ~(1<<9);
	
	//NVIC 
	NVIC_EnableIRQ(USART3_4_IRQn);
	
	
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		//Transmit a character
		//HAL_Delay(1500);
		//transmitChar('a');

		//Transmit a string
		//transmitString("Samuel");
		
		
		//PART ONE CHECKOFF------------------------------
		//char returnedchar = recievedChar();
		//switch(returnedchar){
			//case 'g':
				//transmitChar('g');
				//GPIOC->ODR ^= (1<<9);
			//	break;
		//	case 'r':
				//transmitChar('r');
			//	GPIOC->ODR ^= (1<<6);
			//	break;
			//case 'o':
				//transmitChar('o');
			//	GPIOC->ODR ^= (1<<8);
			//	break;
			//case 'b':
				//transmitChar('b');
			//	GPIOC->ODR ^= (1<<7);
			//	break;
			//default:
			//		transmitString("Error. Must click correct color");	
		//}
		//--------------------------------------------------	
		
		//Part 2 Checkoff -----------------------------------
		int color = 0;
		transmitString("\n\rCMD: ");
		
		//wait for color input
		while(!colorFlag){}
		actionFlag = 0;

		//first switch statment determines the color and sets the number	
		switch(input){
			case 'g':
				transmitString("Green ");
				color = 9;
				break;
			case 'r':
				transmitString("Red ");
				color = 6;
				break;
			case 'o':
				transmitString("Orange ");
				color = 8;
				break;
			case 'b':
				transmitString("Blue ");
				color = 7;
				break;
			default:
					transmitString("Error. Must click correct color");
			
		}
		
		//wait for action input
		while(!actionFlag){}
		
		//As long as a color is input correctly this will determine the action
		if(color != 0)
		{
			switch(input){
			case '0':
				transmitString("OFF ");
				GPIOC->ODR &= ~(1<<color);
				break;
			case '1':
				transmitString("ON ");
				GPIOC->ODR |= (1<<color);
				break;
			case '2':
				transmitString("Toggle ");
				GPIOC->ODR ^= (1<<color);
				break;
			default:
					transmitString("Error. Must click correct powermode ");
			
		}
		}
		
		//Reset Flags
		actionFlag = 0;	
		colorFlag = 0;
		

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}




void USART3_4_IRQHandler(void)
{
	input = recievedChar();
	colorFlag = 1;
	actionFlag = 1;
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
