

#include "main.h"
#include "lcd.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_dma.h"
#include "FONT.H"

 SPI_HandleTypeDef hspi1;

 DMA_HandleTypeDef hdma_spi1_tx;

 DMA_HandleTypeDef hdma_spi1_rx;


 uint16_t bars_x[64] = {
      5,   9,  20,  24,  35,  39,  50,  54,
     65,  69,  80,  84,  95,  99, 110, 114,
    125, 129, 140, 144, 155, 159, 170, 174,
    185, 189, 200, 204, 215, 219, 230, 234,
    245, 249, 260, 264, 275, 279, 290, 294,
    305, 309, 320, 324, 335, 339, 350, 354,
    365, 369, 380, 384, 395, 399, 410, 414,
    425, 429, 440, 444, 455, 459, 470, 474
 };
 uint16_t colors[8] = {
   0x00FF,  //
   0xFF00,  //
   0xF0F0,
   0x0F0F,
   0xAAAA,
   0xA0A0,
   0x0A0A,
   0xAA00
 };
uint16_t DMA_MIN_SIZE = 32;
uint16_t disp_buf[320 * 5];

void SPI_init(void) {
  // Use the LCD_CS_Pin macro instead of a literal pin number.
  HAL_GPIO_WritePin(GPIOA, LCD_CS_Pin, GPIO_PIN_SET);
}

void LCD_reset(void) {
  // Use LCD_RST_Pin macro.
  HAL_GPIO_WritePin(GPIOA, LCD_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOA, LCD_RST_Pin, GPIO_PIN_SET);
}

void transferComm(uint8_t send_data) {
  // Set RS low (command mode) and use LCD_CS_Pin for chip select.
  HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, LCD_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &send_data, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, LCD_CS_Pin, GPIO_PIN_SET);
}

void transferData(uint8_t send_data) {
  // Set RS high (data mode).
  HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, LCD_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &send_data, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, LCD_CS_Pin, GPIO_PIN_SET);
}

void LCD_init(void) {
  transferComm(0xF0); // Command set control
  transferData(0xC3); // Enable command 2 part I

  transferComm(0xF0);
  transferData(0x96); // Enable command 2 part II

  transferComm(0x36); // Memory data access control
  transferData(0x48);
  // Set row address order, column order, exchange, refresh direction, etc.

  transferComm(0x3A); // Interface pixel format
  transferData(0x05); // 16 bits per pixel

  transferComm(0xB0); // Interface mode control
  transferData(0x80);

  transferComm(0xB6); // Display function control
  transferData(0x00);
  transferData(0x02);

  transferComm(0xB5); // Blanking porch control
  transferData(0x02);
  transferData(0x03);
  transferData(0x00);
  transferData(0x04);

  transferComm(0xB1); // Frame rate control
  transferData(0x80);
  transferData(0x10);

  transferComm(0xB4); // Display inversion control
  transferData(0x00);

  transferComm(0xB7); // Entry mode set
  transferData(0xC6);

  transferComm(0xC5); // VCOM control
  transferData(0x1C);

  transferComm(0xE4);
  transferData(0x31);

  transferComm(0xE8); // Display output control adjust
  transferData(0x40);
  transferData(0x8A);
  transferData(0x00);
  transferData(0x00);
  transferData(0x29);
  transferData(0x19);
  transferData(0xA5);
  transferData(0x33);

  transferComm(0xC2); // Power control 3
  transferComm(0xA7);

  transferComm(0xE0); // Positive gamma control
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

  transferComm(0xE1); // Negative gamma control
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

  transferComm(0xF0);
  transferData(0x3C);

  transferComm(0xF0);
  transferData(0x69);

  transferComm(0x13); // Normal display mode ON

  transferComm(0x11); // Exit sleep mode

  transferComm(0x29); // Display ON
}

void drawBackground(uint8_t color1, uint8_t color2) {
  uint32_t numPixels = 320UL * 480UL;
  transferComm(0x2A); // Column address set
  transferData(0x00);
  transferData(0x00);
  transferData(0x01);
  transferData(0x3F);

  transferComm(0x2B); // Row address set
  transferData(0x00);
  transferData(0x00);
  transferData(0x01);
  transferData(0xDF);

  transferComm(0x2C); // Memory write
  for(uint32_t i = 0; i < numPixels; i++){
    transferData(color1);
    transferData(color2);
  }
}

void drawRectangle(uint16_t H_start, uint16_t H_end, uint16_t V_start, uint16_t V_end, uint8_t color1, uint8_t color2) {
  uint32_t H_width = H_end - H_start;
  uint32_t V_width = V_end - V_start;

  uint16_t H_start_fixed = H_start;
  uint16_t V_start_fixed = V_start;
  uint16_t H_end_fixed = H_start_fixed + H_width - 1;
  uint16_t V_end_fixed = V_start_fixed + V_width - 1;

  uint8_t H_start_1 = H_start_fixed >> 8;
  uint8_t H_start_2 = H_start_fixed & 0xFF;
  uint8_t H_end_1   = H_end_fixed >> 8;
  uint8_t H_end_2   = H_end_fixed & 0xFF;

  uint8_t V_start_1 = V_start_fixed >> 8;
  uint8_t V_start_2 = V_start_fixed & 0xFF;
  uint8_t V_end_1   = V_end_fixed >> 8;
  uint8_t V_end_2   = V_end_fixed & 0xFF;

  transferComm(0x2A); // Column address set
  transferData(V_start_1);
  transferData(V_start_2);
  transferData(V_end_1);
  transferData(V_end_2);

  transferComm(0x2B); // Row address set
  transferData(H_start_1);
  transferData(H_start_2);
  transferData(H_end_1);
  transferData(H_end_2);

  transferComm(0x2C); // Memory write
  for(uint32_t i = 0; i < (H_width + 1) * (V_width + 1); i++){
    transferData(color1);
    transferData(color2);
  }
}

void drawBetterBar(uint16_t bar_x, uint16_t bar_y, uint8_t color1, uint8_t color2) {
  drawRectangle(bar_x, bar_x , 0, bar_y, color1, color2);
}

void eraseBar(uint16_t bar_x, uint16_t bar_y) {
  drawRectangle(bar_x, bar_x , bar_y, MAX_SCREEN_HEIGHT, 0x00, 0x00);
}
void LCD_Fill_Color(uint16_t color) {
    // Set the address window for the entire screen.
    // Columns: 0 to 319 (0x0000 to 0x013F)
    transferComm(0x2A);
    transferData(0x00);   // Column start high byte
    transferData(0x00);   // Column start low byte
    transferData(0x01);   // Column end high byte (0x01)
    transferData(0x3F);   // Column end low byte (0x3F = 63, so 0x013F = 319)

    // Rows: 0 to 479 (0x0000 to 0x01DF)
    transferComm(0x2B);
    transferData(0x00);   // Row start high byte
    transferData(0x00);   // Row start low byte
    transferData(0x01);   // Row end high byte (0x01)
    transferData(0xDF);   // Row end low byte (0xDF = 223, so 0x01DF = 479)

    // Begin writing pixel data.
    transferComm(0x2C);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	// Set LCD_CS pin low to use LCD

    // Pre-fill the DMA buffer.
    // disp_buf is declared as: uint16_t disp_buf[320 * HOR_LEN];
    // HOR_LEN defines the number of rows processed per DMA transfer.
    uint32_t totalPixelsPerChunk = 320UL * 5;
    for (uint32_t i = 0; i < totalPixelsPerChunk; i++) {
        disp_buf[i] = color;
    }

    // Calculate how many chunks (each of HOR_LEN rows) are needed.
    uint32_t numChunks = 480UL / 5;
    for (uint32_t i = 0; i < numChunks; i++) {
        dma_write_data((uint8_t*)disp_buf, sizeof(disp_buf));
    }
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

}
void dma_write_data(uint8_t *buff, size_t buff_size)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);	// Set LCD_RS pin low to set as COMMAND
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);	// Set LCD_CS pin low to use LCD
	while(buff_size>0)

	{
		uint16_t chunk_size = buff_size > 65535 ? 65535 : buff_size;
		if (DMA_MIN_SIZE <= buff_size)
					{
						if(HAL_SPI_Transmit_DMA(&hspi1, buff, chunk_size))
						{

						}
						while (hspi1.hdmatx->State != HAL_DMA_STATE_READY)
						{}
					}
					else{
						HAL_SPI_Transmit(&hspi1, buff, chunk_size, HAL_MAX_DELAY);
					}
		buff += chunk_size;
		buff_size -= chunk_size;
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}
void LCD_SetAddressWindow(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
	  uint32_t H_width = xEnd - xStart;
	  uint32_t V_width = yEnd - yStart;

	  uint16_t H_start_fixed = xStart;
	  uint16_t V_start_fixed = yStart;
	  uint16_t H_end_fixed = H_start_fixed + H_width - 1;
	  uint16_t V_end_fixed = V_start_fixed + V_width - 1;

	  uint8_t H_start_1 = H_start_fixed >> 8;
	  uint8_t H_start_2 = H_start_fixed & 0xFF;
	  uint8_t H_end_1   = H_end_fixed >> 8;
	  uint8_t H_end_2   = H_end_fixed & 0xFF;

	  uint8_t V_start_1 = V_start_fixed >> 8;
	  uint8_t V_start_2 = V_start_fixed & 0xFF;
	  uint8_t V_end_1   = V_end_fixed >> 8;
	  uint8_t V_end_2   = V_end_fixed & 0xFF;
		transferComm(0x2A); // Column address set
		transferData(V_start_1);
		transferData(V_start_2);
		transferData(V_end_1);
		transferData(V_end_2);

		transferComm(0x2B); // Row address set
		transferData(H_start_1);
		transferData(H_start_2);
		transferData(H_end_1);
		transferData(H_end_2);

    // Write memory
    transferComm(0x2C);
}
void drawRectangleDMA(uint16_t xStart, uint16_t xEnd, uint16_t yStart, uint16_t yEnd, uint16_t color)
{
    // Calculate width and height (inclusive of both endpoints)
    uint32_t width  = xEnd - xStart + 1;
    uint32_t height = yEnd - yStart + 1;
    uint32_t totalPixels = width * height;

    // Set the LCD's address window for the rectangle.
    LCD_SetAddressWindow(xStart, yStart, xEnd, yEnd);

    // Define a chunk size for the DMA transfer.
    // You can adjust CHUNK_SIZE based on available memory and performance.
    #define CHUNK_SIZE 100
    uint16_t chunkBuffer[CHUNK_SIZE];

    // Fill the chunk buffer with the specified color.
    for (uint16_t i = 0; i < CHUNK_SIZE; i++) {
        chunkBuffer[i] = color;
    }

    // Calculate how many full chunks and remaining pixels need to be sent.
    uint32_t fullChunks = totalPixels / CHUNK_SIZE;
    uint32_t remainder  = totalPixels % CHUNK_SIZE;

    // Send full chunks using DMA.
    for (uint32_t i = 0; i < fullChunks; i++) {
        dma_write_data((uint8_t*)chunkBuffer, CHUNK_SIZE * sizeof(uint16_t));
    }

    // Send any remaining pixels.
    if (remainder > 0) {
        dma_write_data((uint8_t*)chunkBuffer, remainder * sizeof(uint16_t));
    }
}

void updateBarGraph(int32_t *barHeights)
{
    static int32_t prevHeights[NUM_BARS] = {0};

    for (uint8_t i = 0; i < NUM_BARS; i++)
    {
        uint16_t xStart = bars_x[2*i];
        uint16_t xEnd   = bars_x[2*i+1];

        int32_t oldH = prevHeights[i];
        int32_t newH = barHeights[i];

        // clamp heights
        if (newH < 0) newH = 0;
        if (newH > MAX_SCREEN_HEIGHT) newH = MAX_SCREEN_HEIGHT;
        if (oldH < 0) oldH = 0;
        if (oldH > MAX_SCREEN_HEIGHT) oldH = MAX_SCREEN_HEIGHT;

        uint16_t yOldTop = (oldH == 0)
                          ? (MAX_SCREEN_HEIGHT - 1)
                          : (MAX_SCREEN_HEIGHT - oldH);
        uint16_t yNewTop = (newH == 0)
                          ? (MAX_SCREEN_HEIGHT - 1)
                          : (MAX_SCREEN_HEIGHT - newH);

        // pick which color‐slot based on bar index
        // each color covers 4 bars
        uint8_t colorSlot = (i / 4) % (sizeof(colors)/sizeof(colors[0]));
        uint16_t drawColor = colors[colorSlot];

        if (newH < oldH)
        {
            // erase the gap in BLACK
            drawRectangleDMA(
              xStart, xEnd,
              yOldTop,
              yNewTop,
              BLACK
            );
        }
        else if (newH > oldH)
        {
            // draw the extension in the selected color
            drawRectangleDMA(
              xStart, xEnd,
              yNewTop,
              yOldTop,
              drawColor
            );
        }

        prevHeights[i] = newH;
    }
}
