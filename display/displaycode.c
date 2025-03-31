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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



#define F_CPU 16000000UL
#define NUM_BARS 11
#define BAR_WIDTH 5
#define BAR_HEIGHT 1
#define MAX_SCREEN_HEIGHT 320
#define MIN_SCREEN_HEIGHT 0



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */



void SPI_init(void);
void LCD_reset(void);
void transferComm(uint8_t send_data);
void transferData(uint8_t send_data);
void LCD_init(void);
void drawBackground(uint8_t color1, uint8_t color2);
void drawRectangle(uint16_t H_start, uint16_t H_end, uint16_t V_start, uint16_t V_end, uint8_t color1, uint8_t color2);
//void drawBar(uint16_t bar_x, uint16_t bar_y, uint8_t color1, uint8_t color2);
void drawBetterBar(uint16_t bar_x, uint16_t bar_y, uint8_t color1, uint8_t color2);
void eraseBar(uint16_t bar_x, uint16_t bar_y);



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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */



  uint8_t background1 = 0x00;
  uint8_t background2 = 0x00;

  // BAR TEST initialize bar variables
  //uint32_t bar_x = 40;
  //uint32_t bar_y = 0;

  // BETTER BAR TEST initialize bar variables
  uint16_t bar_x[NUM_BARS] = {40, 80, 120, 160, 200, 240, 280, 320, 360, 400, 440};
  uint16_t bar_y[NUM_BARS] = {310, 70, 120, 0, 30, 110, 300, 250, 30, 130, 90};
  uint16_t color1[NUM_BARS] = {0xF8, 0xFA, 0xFD, 0xFE, 0xFF, 0x86, 0x04, 0x02, 0x00, 0x20, 0x48};
  uint16_t color2[NUM_BARS] = {0x00, 0x80, 0x20, 0x80, 0xE0, 0x00, 0x00, 0x10, 0x1F, 0x18, 0x10};
  uint16_t direction[NUM_BARS] = {1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1};

  SPI_init();

  LCD_reset();

  // delay for power up sequence
  HAL_Delay(100);

  LCD_init();

  drawBackground(background1, background2);



  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	// BAR TEST
	for (int i = 0; i < NUM_BARS; i++) {
		uint16_t old_y = bar_y[i]; // Keep log of the old bar's height position
		//bar_y[i] = (bar_y[i] + BAR_HEIGHT) % MAX_SCREEN_HEIGHT; //FOR A FULLY RESETTING BAR A TOP OF SCREEN

		// FOR BACK AND FORTH MOTION OF BAR
		// WOULD DELETE FOR ACTUAL AUDIO IMPLEMENATION
		if (bar_y[i] > MAX_SCREEN_HEIGHT) {
			direction[i] = 0;
		}
		if (bar_y[i] == MIN_SCREEN_HEIGHT) {
			direction[i] = 1;
		}

		// Make bar travel up or down depending on direction[i] value
		// WOULD DELETE FOR ACTUAL AUDIO IMPLEMENATION
		if (direction[i] == 1) {
			bar_y[i] += BAR_HEIGHT;
		}
		else {
			bar_y[i] -= BAR_HEIGHT;
		}

		// Erase if bar went down, draw when bar went up
		if (bar_y[i] > old_y) {
			drawRectangle(bar_x[i], bar_x[i] + BAR_WIDTH, old_y, bar_y[i], color1[i], color2[i]);
		}
		else if (old_y >= bar_y[i]) {
			drawRectangle(bar_x[i], bar_x[i] + BAR_WIDTH, bar_y[i], old_y, 0x00, 0x00);
	    }
	}

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin|LCD_RST_Pin|LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RS_Pin LCD_RST_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_RST_Pin|LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



void SPI_init(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); 	// Set LCD_CS pin high (low means use LCD, done later)
}

void LCD_reset(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); 	// Set LCD_RST pin low to reset LCD
	HAL_Delay(1000); 										// Wait a second to recognize pin state
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   	// Set LCD_RST pin high so LCD is active
}

void transferComm(uint8_t send_data) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);	// Set LCD_RS pin low to set as COMMAND
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	// Set LCD_CS pin low to use LCD

	HAL_SPI_Transmit(&hspi1, &send_data, 1, HAL_MAX_DELAY);	// Send the send_data data, also waits for transmission complete

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);		// Set LCD_CS pin high since LCD no longer used
}

void transferData(uint8_t send_data) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);		// Set LCD_RS pin high to set as DATA
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	// Set LCD_CS pin low to use LCD

	HAL_SPI_Transmit(&hspi1, &send_data, 1, HAL_MAX_DELAY);	// Send the send_data data, also waits for transmission complete

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);		// Set LCD_CS pin high since LCD no longer used
}

void LCD_init(void) { //THE BIG ONE WITH THE WEIRD PRE-WRITTEN CODE
	transferComm(0xF0); // command set control
	transferData(0xC3); // enable command 2 part I

	transferComm(0xF0); // command set control
	transferData(0x96); // enable command 2 part II

	transferComm(0x36); // memory data access control
	transferData(0x48);
	// row address order = 0
	// column address order = 1
	// row/column exchange = 0
	// LCD vertical refresh top to bottom (rather than bottom to top)
	// BGR color filter panel (rather than RGB)
	// horizontal direction left to right (rather than right to left)

	transferComm(0x3A); // interface pixel format
	transferData(0x05); // 16 bit per pixel

	transferComm(0xB0); // interface mode control
	transferData(0x80);
	// DIN/SDA pin used for 3/4 wire serial interface, DOUT pin not used
	// high enable for RGB interface
	// data fetched at rising time
	// low level horizontal sync clock
	// low level vertical sync clock

	transferComm(0xB6); // display function control
	transferData(0x00);
	// display data path to memory (rather than shift register)
	// RGB DE mode (rather than SYNC mode)
	// write display data to GRAM using system interface (rather than RGB)
	// normal scan
	// source output to non display area is V63
	transferData(0x02);
	// gate output scan direction G1 -> G480
	// source output scan direction S1 -> S960
	// gate output sequence G1, G2, G3... G479, G480

	transferComm(0xB5); // blanking porch control
	transferData(0x02); // vertical front porch = 2
	transferData(0x03); // vertical back porch = 3
	transferData(0x00); // just 0
	transferData(0x04); // horizontal back porch = 4

	transferComm(0xB1); // frame rate control
	transferData(0x80); // inversion mode = Fosc
	transferData(0x10); // fps = 91? see calc

	transferComm(0xB4); // display inversion control
	transferData(0x00); // column inversion

	transferComm(0xB7); // entry mode set
	transferData(0xC6);
	// store in GRAM r(0) = b(0) = g(0)
	// deep standby mode off? normal display

	transferComm(0xC5); // VCOM control
	transferData(0x1C); // VCOM = 1.0

	transferComm(0xE4); // ???
	transferData(0x31);

	transferComm(0xE8); // display output control adjust
	transferData(0x40); // always
	transferData(0x8A); // always
	transferData(0x00); // always
	transferData(0x00); // always
	transferData(0x29); // 1.5 * 9 + 9 = 22.5 ms
	transferData(0x19); // G_START = 25
	transferData(0xA5); // G_END = 37, gate driver EQ function ON
	transferData(0x33); // always

	transferComm(0xC2); // power control 3
	transferComm(0xA7);
	// source driving current level = low
	// gamma driving current level = high

	transferComm(0xE0); // positive gamma control
	transferData(0xF0);
	transferData(0x09);
	transferData(0x13);
	transferData(0x12);
	transferData(0x12);
	transferData(0x2B);
	transferData(0x3C);
	transferData(0x44);
	transferData(0x4B);
	transferData(0x1B);
	transferData(0x18);
	transferData(0x17);
	transferData(0x1D);
	transferData(0x21);

	transferComm(0xE1); // negative gamma control
	transferData(0xF0);
	transferData(0x09);
	transferData(0x13);
	transferData(0x0C);
	transferData(0x0D);
	transferData(0x27);
	transferData(0x3B);
	transferData(0x44);
	transferData(0x4D);
	transferData(0x0B);
	transferData(0x17);
	transferData(0x17);
	transferData(0x1D);
	transferData(0x21);

	transferComm(0xF0); // command set control
	transferData(0x3C); // disable command 2 part I

	transferComm(0xF0); // command set control
	transferData(0x69); // disable command 2 part II

	transferComm(0x13); // normal display mode ON

	transferComm(0x11); // turns off sleep mode

	transferComm(0x29); // display ON
}

void drawBackground(uint8_t color1, uint8_t color2) {
	uint32_t numPixels = (uint32_t)320 * (uint32_t)480;

	transferComm(0x2A); // CASET 0-319
	// 0000h = 0
	transferData(0x00);
	transferData(0x00);
	// 013Fh = 319
	transferData(0x01);
	transferData(0x3F);

	transferComm(0x2B); // RASET 0-479
	// 0000h = 0
	transferData(0x00);
	transferData(0x00);
	// 01DFh = 479
	transferData(0x01);
	transferData(0xDF);

	// turn the screen to the background color provided
	transferComm(0x2C); // memory write (start drawing pixels)
	for(uint32_t i = 0; i < numPixels; i++){
		// send first color byte
		transferData(color1);
		// send second color byte
		transferData(color2);
	}
}

void drawRectangle(uint16_t H_start, uint16_t H_end, uint16_t V_start, uint16_t V_end, uint8_t color1, uint8_t color2) {
	uint32_t H_width = H_end - H_start;
	uint32_t V_width = V_end - V_start;

	uint16_t H_start_fixed = H_start;
	//uint16_t V_start_fixed = 320 - V_width - V_start;
	uint16_t V_start_fixed = V_start;

	uint16_t H_end_fixed = H_start_fixed + H_width - 1;
	uint16_t V_end_fixed = V_start_fixed + V_width - 1;

	uint8_t H_start_1 = H_start_fixed >> 8;
	uint8_t H_start_2 = H_start_fixed >> 0;
	uint8_t H_end_1 = H_end_fixed >> 8;
	uint8_t H_end_2 = H_end_fixed >> 0;

	uint8_t V_start_1 = V_start_fixed >> 8;
	uint8_t V_start_2 = V_start_fixed >> 0;
	uint8_t V_end_1 = V_end_fixed >> 8;
	uint8_t V_end_2 = V_end_fixed >> 0;

	transferComm(0x2A); // CASET
	transferData(V_start_1);
	transferData(V_start_2);
	transferData(V_end_1);
	transferData(V_end_2);

	transferComm(0x2B); // RASET
	transferData(H_start_1);
	transferData(H_start_2);
	transferData(H_end_1);
	transferData(H_end_2);

	// turn the region to the color provided
	transferComm(0x2C); // memory write (start drawing pixels)
	for(uint32_t i = 0; i < (H_width + 1) * (V_width + 1); i++){
		// send first color byte
		transferData(color1);
		// send second color byte
		transferData(color2);
	}
}

/*void drawBar(uint16_t bar_x, uint16_t bar_y, uint8_t color1, uint8_t color2) {
	if(bar_y + BAR_HEIGHT < MAX_SCREEN_HEIGHT) {
		drawRectangle(bar_x, bar_x + BAR_WIDTH, bar_y, bar_y + BAR_HEIGHT, color1, color2); // CREATE NEW BOX
		HAL_Delay(20);
		bar_y += BAR_HEIGHT;
		drawBar(bar_x, bar_y, color1, color2);
	}
	bar_y -= BAR_HEIGHT;
	drawRectangle(bar_x, bar_x + BAR_WIDTH, bar_y, bar_y + BAR_HEIGHT, 0x00, 0x00); // ERASE BOX BEFORE
	HAL_Delay(20);
}*/

void drawBetterBar(uint16_t bar_x, uint16_t bar_y, uint8_t color1, uint8_t color2) {
	drawRectangle(bar_x, bar_x + BAR_WIDTH, 0, bar_y, color1, color2);
}

void eraseBar(uint16_t bar_x, uint16_t bar_y) {
	drawRectangle(bar_x, bar_x + BAR_WIDTH, bar_y, MAX_SCREEN_HEIGHT, 0x00, 0x00);
}



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
