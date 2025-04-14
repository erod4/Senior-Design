/*
 * display.h
 *
 *  Created on: Mar 30, 2025
 *      Author: enrique
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_
#include <inttypes.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"
#include "spi.h"
#include "arm_math.h"

#define NUM_BARS			16
#define BAR_WIDTH			20
#define BAR_SPACING 		5
#define MAX_BARS 			16
#define BAR_GAP           	2

#define MAX_SCREEN_HEIGHT	320
#define MIN_SCREEN_HEIGHT	0

#define MAX_SCREEN_WIDTH	480
#define MIN_SCREEN_WIDTH	0

#define BG1 				0x00
#define BG2 				0x00

#define CHUNK_SIZE     160  // For example, one line or a small chunk
void drawBackground_DMA(uint8_t colorHigh, uint8_t colorLow);
void LCD_reset(void);

void LCD_init(void);

void drawBackground(uint8_t color1, uint8_t color2);

void drawRectangle(uint16_t H_start, uint16_t H_end, uint16_t V_start, uint16_t V_end, uint8_t color1, uint8_t color2);
//void drawBar(uint16_t bar_x, uint16_t bar_y, uint8_t color1, uint8_t color2);
void drawBetterBar(uint16_t bar_x, uint16_t bar_y, uint8_t color1, uint8_t color2);

void eraseBar(uint16_t bar_x, uint16_t bar_y);

void updateSpectrumDisplay(const int32_t *newValues, uint16_t numBars, uint8_t barColor1, uint8_t barColor2);

void LCD_Fill_Color(uint16_t color);
void dma_write_data(uint8_t *buff, size_t buff_size);
#endif /* INC_DISPLAY_H_ */
