/*
 * display.c
 *
 *  Created on: Mar 30, 2025
 *      Author: enrique
 */


#include "display.h"
#include "main.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"
#include "stm32g4xx_hal_gpio.h"
#include "spi.h"
#include <string.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
uint16_t previous_y[MAX_BARS] = { 0 };
uint16_t DMA_MIN_SIZE = 16;
uint16_t disp_buf[320 * 5];
void LCD_reset(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); 	// Set LCD_RST pin low to reset LCD
	HAL_Delay(1000); 										// Wait a second to recognize pin state
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   	// Set LCD_RST pin high so LCD is active
}


void LCD_init(void) {
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

void drawBetterBar(uint16_t bar_x, uint16_t bar_y, uint8_t color1, uint8_t color2) {
	drawRectangle(bar_x, bar_x + BAR_WIDTH, 0, bar_y, color1, color2);
}

void eraseBar(uint16_t bar_x, uint16_t bar_y) {
	drawRectangle(bar_x, bar_x + BAR_WIDTH, bar_y, MAX_SCREEN_HEIGHT, 0x00, 0x00);
}

void updateSpectrumDisplay(const int32_t *newValues, uint16_t numBars, uint8_t barColor1, uint8_t barColor2)
{
    for (uint16_t i = 0; i < numBars; i++)
    {
        // Calculate the horizontal position for this bar.
        uint16_t bar_x = i * (BAR_WIDTH + BAR_GAP);

        // New height for this bar.
        uint16_t newHeight = (uint16_t)newValues[i];
        // Previous height for this bar.
        uint16_t prevHeight = previous_y[i];

        if (newHeight > prevHeight)
        {
            // Bar has grown:
            // Draw the additional portion from prevHeight to newHeight.
            // V_start = prevHeight, V_end = newHeight.
            // This assumes that V=0 is at the top and increasing V goes downward.
            drawRectangle(bar_x, bar_x , prevHeight, newHeight, barColor1, barColor2);
        }
        else if (newHeight < prevHeight)
        {
            // Bar has shrunk:
            // Erase the area from newHeight to prevHeight.
            // Erase by drawing a black rectangle (color bytes 0x00) over the difference.
            eraseBar(bar_x, newHeight);
        }

        // Update the stored previous height.
        previous_y[i] = newHeight;
    }
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
							printf("Error\r\n");
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
