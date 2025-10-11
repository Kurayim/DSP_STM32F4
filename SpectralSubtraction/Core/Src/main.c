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




#define FRAME_LEN 			1024
#define HALF_FRAME			(FRAME_LEN / 2)
#define NOISE_SEC			4
#define SAMP_RATE			22050

// Select Low Pass Filter with FIR OR IIR
#define FFT_METHOD		1
#define IFFT_METHOD		0
#define FILTER_M		FFT_METHOD

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
bool LetOverlapFrame	= false;


WAV_StatusTypeDef ResultWave;
MAT_StatusTypeDef ResultMat;

arm_rfft_fast_instance_f32 S;



uint8_t BufferReceive[64];


float32_t alpha = 3.5f;
float32_t Gain = 1.2f;
float32_t maxSample = 0.0f;
float32_t IndexFrame = 0;


float32_t wav_in[HALF_FRAME + 1];   // buffer to read from SD (int16)
float32_t Frame[FRAME_LEN + 1];
float32_t CleanSpectrum[HALF_FRAME + 1];
float32_t FFTOut[FRAME_LEN + 1];
float32_t IFFTIn[FRAME_LEN + 1];
float32_t CleanSampleNew[FRAME_LEN + 1];
float32_t CleanSampleOld[HALF_FRAME + 1];
float32_t wav_out[FRAME_LEN + 1];  // buffer to write to SD (int16)
float32_t fftMagnitude[(FRAME_LEN/2)+1];
float32_t HammingWin[FRAME_LEN+1];
float32_t NoiseSpectrum[(FRAME_LEN/2)+1];
float32_t SignalMagnitud[(FRAME_LEN/2)+1];
float32_t SignalPhase[(FRAME_LEN/2)+1];
float32_t WinSum[(FRAME_LEN+HALF_FRAME)+1];

float32_t buffd[1000] = {'\0'};
uint32_t  ind = 0;
float32_t sumd = 0.0f;


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

void MagniNormal(float32_t *buf, uint32_t numSample){
	float32_t norMagn = 2.0f / numSample;
	int i=0;
	*buf = *buf / numSample;
	buf++;
	i++;
	while(i<numSample){
		*buf = *buf * norMagn;
		buf++;
		i++;
	}
}

bool generateHammingWindow(float32_t *window_array, int window_size) {
    if (window_size <= 0)
        return false;

    for (int n = 0; n < window_size; n++) {
        window_array[n] = 0.54f - 0.46f * arm_cos_f32((2.0f * M_PI * n) / (window_size - 1));
    }
    return true;
}

void CopyFloatArray(float32_t *str1_in, float32_t *str2_out, uint16_t size){
	for(int i = 0 ; i<size ; i++){
		*str2_out = *str1_in;
		str2_out++;
		str1_in++;
	}
}
void ResetArray(float32_t *buff, uint16_t size){
	for(int i = 0 ; i<size ; i++){
		*buff = 0;
		buff++;
	}
}
void WindowApply(float32_t *buf, float32_t *Win, uint16_t size){
  for (uint32_t i = 0; i < size; i++) {
	  (*buf) = (*buf) * (*Win);
	  buf++;
	  Win++;
  }
}
void SumArray(float32_t *sum, float32_t *add, uint16_t size){
  for (uint32_t i = 0; i < size; i++) {
	  (*sum) = (*sum) + (*add);
	  sum++;
	  add++;
  }
}
void SimpleCalArray(uint8_t oper, float32_t *res, float32_t *a, float32_t *b, uint16_t size){
  for (uint32_t i = 0; i < size; i++) {
	  switch (oper) {
	      case 1:
	    	  (*res) = (*a) + (*b);
	          break;
	      case 2:
	    	  (*res) = (*a) / (*b);
	          break;
	      case 3:
	    	  (*res) = (*a) - (*b);
	      	  break;
	      case 4:
	    	  (*res) = (*a) * (*b);
	      	  break;
	      default:
	          return;
	          break;
	  }
	  a++;
	  b++;
	  res++;
  }
}
void DivisionArray(float32_t *buf, float32_t div, uint16_t size){
  for (uint32_t i = 0; i < size; i++) {
	  (*buf) = (*buf) / (float32_t)(div);
	  buf++;
  }
}
void ComputePhase(float32_t *complex_in, float32_t *phase_out, uint16_t size){
  float32_t imag = 0.0f , real = 0.0f;
  for (uint32_t i = 0; i < size; i++) {
	  real = *complex_in;
	  complex_in++;
	  imag = *complex_in;
	  complex_in++;
	  (*phase_out) =atan2f(imag, real);
	  phase_out++;
  }
}
void CleanMagnitude(float32_t *out, float32_t *sig, float32_t *noise, uint16_t size){
	for (uint32_t i = 0; i < size; i++) {
	  (*out) = (*sig) - (alpha * (*noise));
	  noise++;
	  sig++;
	  out++;
  }
}
void ApplySpectralFloor(float32_t *clean_mag, float32_t *spectral_floor, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
    	*spectral_floor = *spectral_floor * 0.02f;
        if (*clean_mag < *spectral_floor) {
            *clean_mag = *spectral_floor;
        }
        clean_mag++;
        spectral_floor++;
    }
}
void ComplexConvertPolarToAlgebraic(float32_t *frame_freq, float32_t *mag, float32_t *phase, uint16_t size) {

	for(uint16_t i = 0; i < size; i++){
		*frame_freq = *mag * arm_cos_f32(*phase); // real part
		frame_freq++;
		*frame_freq = (*mag) * arm_sin_f32(*phase); // imag part
		frame_freq++;
		phase++;
		mag++;
    }
}
float32_t FindMax(float32_t *buf, uint16_t size){
  float32_t max = 0.0f;
  for (uint32_t i = 0; i < size; i++) {
	  if(*buf > max)
		  max = *buf;
	  buf++;
  }
  return max;
}
void GainApply(float32_t *buf, float32_t gain, uint16_t size){
  for (uint32_t i = 0; i < size; i++) {
	  (*buf) = (*buf) * gain;
	  buf++;
  }
}
void LimitClipp(float32_t* frame_time, uint16_t size) {
    for(uint32_t i = 0; i < size; i++) {
        if(*frame_time > 1.0f) {
            *frame_time = 1.0f;
        } else if(*frame_time < -1.0f) {
            *frame_time = -1.0f;
        }
        frame_time++;
    }
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

  // FFT initialization
  arm_rfft_fast_init_f32(&S, FRAME_LEN);
  generateHammingWindow(HammingWin, FRAME_LEN);
  memset(WinSum, '\0', FRAME_LEN+1);
  SimpleCalArray(4, WinSum, HammingWin, HammingWin, FRAME_LEN);
  CopyFloatArray(&WinSum[512], &WinSum[1024], HALF_FRAME);
  SimpleCalArray(1, &WinSum[512], WinSum, &WinSum[1024], HALF_FRAME);


//  sd_unmount();
  HAL_Delay(200);
  sd_mount();


  uint32_t NoiseNumSam = NOISE_SEC * SAMP_RATE;
  float32_t NumFrame = ((NoiseNumSam - FRAME_LEN) / HALF_FRAME) + 1;
  float32_t IndexFrame   = NumFrame;


  ResultWave = WAVFIL_Start_Read( "Record_8.wav");
  ResultWave = WAVFIL_Catch_Data(wav_in, &NumByteRead, HALF_FRAME);

  CopyFloatArray(wav_in, Frame, HALF_FRAME);
  ResetArray(NoiseSpectrum, HALF_FRAME);

  while(IndexFrame != 0){

  	  ResultWave = WAVFIL_Catch_Data(wav_in, &NumByteRead, HALF_FRAME);

  	  CopyFloatArray(wav_in, &Frame[512], HALF_FRAME);

  	  WindowApply(Frame, HammingWin, FRAME_LEN);

  	  arm_rfft_fast_f32(&S, Frame, FFTOut, 0);

   	  arm_cmplx_mag_f32(FFTOut, fftMagnitude, HALF_FRAME);

   	  SumArray(NoiseSpectrum, fftMagnitude, HALF_FRAME);

   	  CopyFloatArray(wav_in, Frame, HALF_FRAME);

   	  IndexFrame--;
  }

  DivisionArray(NoiseSpectrum, NumFrame, HALF_FRAME);

  ResultWave = WAVFIL_End_Read();


  HAL_Delay(200);


  LetOverlapFrame = false;
  ResultWave = WAVFIL_Start_Read( "Record_8.wav");
  ResultWave = WAVFIL_Start_Write("Recording_8_dsp.wav", R_WavremainData, 22050);

  NumFrame = (((R_WavremainData / 2) - FRAME_LEN) / HALF_FRAME) + 1;
  IndexFrame = NumFrame;

  ResultWave = WAVFIL_Catch_Data(wav_in, &NumByteRead, HALF_FRAME);

  CopyFloatArray(wav_in, Frame, HALF_FRAME);
  ResetArray(SignalMagnitud, HALF_FRAME);




  while(IndexFrame != 0){

	  ResultWave = WAVFIL_Catch_Data(wav_in, &NumByteRead, HALF_FRAME);

	  CopyFloatArray(wav_in, &Frame[512], HALF_FRAME);

	  WindowApply(Frame, HammingWin, FRAME_LEN);

	  arm_rfft_fast_f32(&S, Frame, FFTOut, 0);

	  arm_cmplx_mag_f32(FFTOut, SignalMagnitud, HALF_FRAME);

	  ComputePhase(FFTOut, SignalPhase, HALF_FRAME);

	  CleanMagnitude(CleanSpectrum, SignalMagnitud, NoiseSpectrum, HALF_FRAME);

	  ApplySpectralFloor(CleanSpectrum, NoiseSpectrum, HALF_FRAME);

//---------------------------------------------

	  ComplexConvertPolarToAlgebraic(IFFTIn, CleanSpectrum, SignalPhase, HALF_FRAME);

	  arm_rfft_fast_f32(&S, IFFTIn, CleanSampleNew, 1);

	  WindowApply(CleanSampleNew, HammingWin, FRAME_LEN);


	  if(LetOverlapFrame){

		  SumArray(CleanSampleNew, CleanSampleOld, HALF_FRAME);
		  SimpleCalArray(2, CleanSampleNew, CleanSampleNew, &WinSum[512], HALF_FRAME);

	  }
	  else{

		  SimpleCalArray(2, CleanSampleNew, CleanSampleNew, WinSum, HALF_FRAME);

	  }

	  maxSample = 0.092862136481524f;
	  DivisionArray(CleanSampleNew, maxSample, HALF_FRAME);
	  GainApply(CleanSampleNew, Gain, HALF_FRAME);
	  LimitClipp(CleanSampleNew, HALF_FRAME);

	  ResultWave = WAVFIL_Give_Write(CleanSampleNew, HALF_FRAME);

	  CopyFloatArray(&CleanSampleNew[512], CleanSampleOld, HALF_FRAME);

	  IndexFrame--;
	  LetOverlapFrame = true;

	  CopyFloatArray(wav_in, Frame, HALF_FRAME);
  }
  if(IndexFrame == 0){

	  SimpleCalArray(2, CleanSampleNew, CleanSampleNew, &WinSum[1024], HALF_FRAME);

	  maxSample = 0.092862136481524f;
	  DivisionArray(CleanSampleNew, maxSample, HALF_FRAME);
	  GainApply(CleanSampleNew, Gain, HALF_FRAME);
	  LimitClipp(CleanSampleNew, HALF_FRAME);

	  ResultWave = WAVFIL_Give_Write(CleanSampleNew, HALF_FRAME);
  }

  ResultWave = WAVFIL_End_Read();
  ResultWave = WAVFIL_End_Write();




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
//		  	  ResultWave = WAVFIL_Start_Write("FFT_REC8.wav", R_WavremainData, 22050);
//		  	if(ResultWave != WAV_OK){
//		  		break;
//		  		FlagKey = false;
//		  	}


		  // Give data from wave file in the sd card
		//  	ResultWave = WAVFIL_Catch_Data(wav_in, &NumByteRead);
		  if(ResultWave != WAV_OK){
			  break;
			  FlagKey = false;
		  }

		  // Trying to connect to MATLAB on the PC
//		  ResultMat = MAT_Connect(22050, 32767, NumByteRead);
//		  if(ResultMat != MAT_OK){
//			  break;
//			  FlagKey = false;
//		  }

	  	  arm_rfft_fast_f32(&S, wav_in, wav_out, 0);
	  	  arm_cmplx_mag_f32(wav_out, fftMagnitude, FRAME_LEN/2);



		  // Storing Filtered data in the sd card
//		  ResultWave = WAVFIL_Give_Write(wav_out);
//		  if(ResultWave != WAV_OK){
//			  break;
//			  FlagKey = false;
//		  }

		  // Send Filtered data to MATLAB
		  ResultMat = MAT_SendSamples(wav_out, FRAME_LEN/2, 32767.0f);
		  if(ResultMat != MAT_OK){
			  break;
			  FlagKey = false;
		  }

		  // We need to continue this until the end of the wavw recording file.
		  while(R_WavremainData != 0){

			  // Give data from wave file in the sd card
			//  ResultWave = WAVFIL_Catch_Data(wav_in, &NumByteRead);
			  if(ResultWave != WAV_OK){
				  break;
				  FlagKey = false;
			  }

			  // Apply Low Pass Filter
	  	  	  arm_rfft_fast_f32(&S, wav_in, wav_out, 0);
	  	  	  arm_cmplx_mag_f32(wav_out, fftMagnitude, FRAME_LEN/2);
	  	  	  MagniNormal(fftMagnitude, FRAME_LEN/2);


			  // Send Filtered data to MATLAB
			  ResultMat = MAT_SendSamples(wav_out, FRAME_LEN/2, 32767.0f);
			  if(ResultMat != MAT_OK){
				  break;
				  FlagKey = false;
			  }

			  // Storing Filtered data in the sd card
//			  ResultWave = WAVFIL_Give_Write(wav_out);
//			  if(ResultWave != WAV_OK){
//				  break;
//				  FlagKey = false;
//			  }
		  }
		  // Send to MATLAB end signal.
		  MAT_EndSignal();
		  // Close the wave recording file.
		  ResultWave = WAVFIL_End_Read();
		  // Close the new wave filtered file.
//		  ResultWave = WAVFIL_End_Write();
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
