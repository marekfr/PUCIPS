/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <sine2.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N 8
#define N1 16
#define N2 32
#define N3 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
;
uint32_t sine_val8[N];
uint32_t sine_val16[N1];
uint32_t sine_val32[N2];
uint32_t sine_val128[N3];
uint32_t sawtooth_val16[N1];
uint32_t sawtooth_val32[N2];
uint32_t sawtooth_val128[N3];

uint8_t i=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void get_sineval_N16_10Hz (){

	__HAL_TIM_SET_PRESCALER(&htim6,99);
	__HAL_TIM_SET_AUTORELOAD(&htim6, 4499);
	for (int i=0;i<N1;i++){
		sine_val16[i] = ((sin(i*2*M_PI/N1)+1)*(3900/2)+60);//+ OFFSET
//		sine_val[i] = ((sin(i*2*M_PI/N)+1)*(4096/2)); // ORYGINAL?
	}
}
void get_sineval_N32_10Hz (){
	__HAL_TIM_SET_PRESCALER(&htim6,99);
	__HAL_TIM_SET_AUTORELOAD(&htim6, 2249);

	for (int i=0;i<N2;i++){
		sine_val32[i] = ((sin(i*2*M_PI/N2)+1)*(3900/2)+60);//+ OFFSET
//		sine_val[i] = ((sin(i*2*M_PI/N)+1)*(4096/2)); // ORYGINAL?
	}
}
void get_sineval_N128_10Hz (){
	__HAL_TIM_SET_PRESCALER(&htim6,9);
	__HAL_TIM_SET_AUTORELOAD(&htim6, 5624);
	for (int i=0;i<N3;i++){
		sine_val128[i] = ((sin(i*2*M_PI/N3)+1)*(3900/2)+60);//+ OFFSET
//		sine_val[i] = ((sin(i*2*M_PI/N)+1)*(4096/2)); // ORYGINAL?
	}
}
void get_sineval_N8_100Hz (){

	__HAL_TIM_SET_PRESCALER(&htim6,71);
	__HAL_TIM_SET_AUTORELOAD(&htim6, 1249);
	for (int i=0;i<N;i++){
		sine_val8[i] = ((sin(i*2*M_PI/N)+1)*(3900/2)+60);//+ OFFSET
//		sine_val[i] = ((sin(i*2*M_PI/N)+1)*(4096/2)); // ORYGINAL?
	}
}
void get_sineval_N16_100Hz (){

	__HAL_TIM_SET_PRESCALER(&htim6,71);
	__HAL_TIM_SET_AUTORELOAD(&htim6, 624);

	for (int i=0;i<N1;i++){
		sine_val16[i] = ((sin(i*2*M_PI/N1)+1)*(3900/2)+60);//+ OFFSET
//		sine_val[i] = ((sin(i*2*M_PI/N)+1)*(4096/2)); // ORYGINAL?
	}
}
void get_sineval_N32_100Hz (){
	__HAL_TIM_SET_PRESCALER(&htim6,11);
	__HAL_TIM_SET_AUTORELOAD(&htim6, 1874);
	for (int i=0;i<N2;i++){
		sine_val32[i] = ((sin(i*2*M_PI/N2)+1)*(3900/2)+60);//+ OFFSET
//		sine_val[i] = ((sin(i*2*M_PI/N)+1)*(4096/2)); // ORYGINAL?
	}
}
void get_sineval_N128_100Hz (){
	__HAL_TIM_SET_PRESCALER(&htim6,0);
	__HAL_TIM_SET_AUTORELOAD(&htim6, 5624);
	for (int i=0;i<N3;i++){
		sine_val128[i] = ((sin(i*2*M_PI/N3)+1)*(3900/2)+60);//+ OFFSET
//		sine_val[i] = ((sin(i*2*M_PI/N)+1)*(4096/2)); // ORYGINAL?
	}
}
void get_sineval_N16_1kHz (){

	__HAL_TIM_SET_PRESCALER(&htim6,0);
	__HAL_TIM_SET_AUTORELOAD(&htim6, 4499);

	for (int i=0;i<N1;i++){
		sine_val16[i] = ((sin(i*2*M_PI/N1)+1)*(3900/2)+60);//+ OFFSET
//		sine_val[i] = ((sin(i*2*M_PI/N)+1)*(4096/2)); // ORYGINAL?
	}
}
void get_sineval_N32_1kHz (){
	__HAL_TIM_SET_PRESCALER(&htim6,0);
	__HAL_TIM_SET_AUTORELOAD(&htim6, 2249);
	for (int i=0;i<N2;i++){
		sine_val32[i] = ((sin(i*2*M_PI/N2)+1)*(3900/2)+60);//+ OFFSET
//		sine_val[i] = ((sin(i*2*M_PI/N)+1)*(4096/2)); // ORYGINAL?
	}
}
void get_sineval_N128_1kHz (){
	// do dokonczenia
	__HAL_TIM_SET_PRESCALER(&htim6,0);
	__HAL_TIM_SET_AUTORELOAD(&htim6, 5624);
	for (int i=0;i<N3;i++){
		sine_val128[i] = ((sin(i*2*M_PI/N3)+1)*(3900/2)+60);//+ OFFSET
//		sine_val[i] = ((sin(i*2*M_PI/N)+1)*(4096/2)); // ORYGINAL?
	}
}
void get_sawtooth_N16_100Hz (){

	__HAL_TIM_SET_PRESCALER(&htim6,71);
	__HAL_TIM_SET_AUTORELOAD(&htim6, 624);
	float A=0;
	for (int i=0;i<N1;i++){
		sawtooth_val16[i] = (A*(3900/2)+60);
		A=A+0.12;
	}
}
void get_sawtooth_N32_100Hz (){
	__HAL_TIM_SET_PRESCALER(&htim6,11);
	__HAL_TIM_SET_AUTORELOAD(&htim6, 1874);
	float A=0;
	for (int i=0;i<N2;i++){
		sawtooth_val32[i] = (A*(3900/2)+60);
		A=A+0.06;
	}
}
void get_sawtooth_N128_100Hz (){
	__HAL_TIM_SET_PRESCALER(&htim6,0);
	__HAL_TIM_SET_AUTORELOAD(&htim6, 5624);
	float A=0;
	for (int i=0;i<N3;i++){
		sawtooth_val128[i] = (A*(3900/2)+60);
		A=A+0.015;
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if(htim->Instance==TIM6)
{

	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, sawtooth_val32[i]);
    i=(i<N2-1) ? (i+1) : (0);

}
}
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
//    get_sineval_N8_100Hz();
//    get_sineval_N16_100Hz();
//    get_sineval_N32_100Hz();
//    get_sineval_N128_100Hz ();
  //  get_sineval_N16_100Hz();
//     get_sineval_N16_10Hz();
//     get_sineval_N32_10Hz();
//     get_sineval_N128_10Hz();
//  get_sawtooth_N16_100Hz();
  get_sawtooth_N32_100Hz();
//  get_sawtooth_N128_100Hz();

  HAL_TIM_Base_Start_IT(&htim6);
//  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, sine_val32, N2, DAC_ALIGN_12B_R);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
