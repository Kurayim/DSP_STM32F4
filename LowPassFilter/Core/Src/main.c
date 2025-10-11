/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "fatfs.h"
#include "usb_device.h"
//
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include "arm_math.h"
#include "math.h"
#include "sd_functions.h"
#include "sd_benchmark.h"
#include "matlab_interface.h"
#include "firCoeffs.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



#define BLOCK_SIZE 			1024
#define FIR_TAP_NUM 		128

// Select Low Pass Filter with FIR OR IIR
#define FIR_METHOD		1
#define IIR_METHOD		0
#define FILTER_M		IIR_METHOD

/* Mohsen Jahed Korime   */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

extern uint32_t R_WavDataNum;
extern uint32_t R_WavremainData;
extern uint16_t R_WavSamRat;
extern uint32_t W_WavremainData;


uint32_t NumByteRead = 0;
uint32_t NumByteWrit = 0;


bool FlagKey 			= false;
bool FalgReadyPacketRx 	= false;


WAV_StatusTypeDef ResultWave;
MAT_StatusTypeDef ResultMat;

#if FILTER_M == FIR_METHOD
	arm_fir_instance_f32 S;
#else
	arm_biquad_cascade_df2T_instance_f32 IIR;
#endif


uint8_t BufferReceive[64];


float32_t wav_buf[WAVE_DAT_RED_SIZ + 1];
float32_t wav_in[BLOCK_SIZE + 1];   // buffer to read from SD (int16)
float32_t wav_out[BLOCK_SIZE + 1];  // buffer to write to SD (int16)
float32_t firState[BLOCK_SIZE + FIR_TAP_NUM - 1];
float32_t biquadState[4 * NUM_STAGES];


//uint8_t bufr[80];
//UINT br;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_0) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
    	FlagKey = true;
    }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */


int _write(int fd, unsigned char *buf, int len) {
  if (fd == 1 || fd == 2) {                     // stdout or stderr ?
    HAL_UART_Transmit(&huart1, buf, len, 999);  // Print to the UART
  }
  return len;
}


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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // Filter initialization
  #if FILTER_M == FIR_METHOD
  	  arm_fir_init_f32(&S, FIR_TAP_NUM, (float32_t *)firCoeffs, firState, BLOCK_SIZE);
  #else
  	  arm_biquad_cascade_df2T_init_f32(&IIR, NUM_STAGES, (float32_t *)biquadCoeffs, (float32_t *)biquadState);
  #endif


  sd_unmount();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



	  while(FlagKey)
	  {

		  sd_mount();
		  // Open recording file
		  ResultWave = WAVFIL_Start_Read( "Recording_8.wav");
		  if(ResultWave != WAV_OK){
			  break;
			  FlagKey = false;
		  }

		  // Create a new wave file
		  #if FILTER_M == FIR_METHOD
		  	  ResultWave = WAVFIL_Start_Write("LPF_FIR.wav", R_WavremainData, 22050);
		  	if(ResultWave != WAV_OK){
		  		break;
		  		FlagKey = false;
		  	}
		  #else
		  	  ResultWave = WAVFIL_Start_Write("LPF_IIR.wav", R_WavremainData, 22050);
		  	if(ResultWave != WAV_OK){
		  		break;
		  		FlagKey = false;
		  	}
		  #endif

		  // Give data from wave file in the sd card
		  WAVFIL_Catch_Data(wav_in, &NumByteRead);
		  if(ResultWave != WAV_OK){
			  break;
			  FlagKey = false;
		  }

		  // Trying to connect to MATLAB on the PC
		  ResultMat = MAT_Connect(22050, 32767, NumByteRead);
		  if(ResultMat != MAT_OK){
			  break;
			  FlagKey = false;
		  }

		  // Apply Low Pass Filter
		  #if FILTER_M == FIR_METHOD
		  	  arm_fir_f32(&S, wav_in, wav_out, NumByteRead);
		  #else
		  	  arm_biquad_cascade_df2T_f32(&IIR, wav_in, wav_out, BLOCK_SIZE);
		  #endif


		  // Storing Filtered data in the sd card
		  ResultWave = WAVFIL_Give_Write(wav_out);
		  if(ResultWave != WAV_OK){
			  break;
			  FlagKey = false;
		  }

		  // Send Filtered data to MATLAB
		  ResultMat = MAT_SendSamples(wav_out, NumByteRead, 32767.5f);
		  if(ResultMat != MAT_OK){
			  break;
			  FlagKey = false;
		  }

		  // We need to continue this until the end of the wavw recording file.
		  while(R_WavremainData != 0){

			  // Give data from wave file in the sd card
			  ResultWave = WAVFIL_Catch_Data(wav_in, &NumByteRead);
			  if(ResultWave != WAV_OK){
				  break;
				  FlagKey = false;
			  }

			  // Apply Low Pass Filter
			  #if FILTER_M == FIR_METHOD
				  arm_fir_f32(&S, wav_in, wav_out, NumByteRead);
			  #else
				  arm_biquad_cascade_df2T_f32(&IIR, wav_in, wav_out, BLOCK_SIZE);
			  #endif

			  // Send Filtered data to MATLAB
			  ResultMat = MAT_SendSamples(wav_out, NumByteRead, 32767.5f);
			  if(ResultMat != MAT_OK){
				  break;
				  FlagKey = false;
			  }

			  // Storing Filtered data in the sd card
			  ResultWave = WAVFIL_Give_Write(wav_out);
			  if(ResultWave != WAV_OK){
				  break;
				  FlagKey = false;
			  }
		  }
		  // Send to MATLAB end signal.
		  MAT_EndSignal();
		  // Close the wave recording file.
		  ResultWave = WAVFIL_End_Read();
		  // Close the new wave filtered file.
		  ResultWave = WAVFIL_End_Write();
		  // Don't come back to this loop before pressing the bottom.
		  FlagKey = false;
	  }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
