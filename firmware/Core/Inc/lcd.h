/*
 * lcd.h
 *
 *  Created on: Apr 5, 2025
 *      Author: enrique
 */

#ifndef LCD_H_
#define LCD_H_

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_dma.h"

extern DMA_HandleTypeDef hdma_spi1_tx;

extern DMA_HandleTypeDef hdma_spi1_rx;
extern SPI_HandleTypeDef hspi1;

#define NUM_BARS            16
#define MAX_SCREEN_HEIGHT   320
#define MIN_SCREEN_HEIGHT   0
#define FONT_WIDTH  		6
#define FONT_HEIGHT 		12
#define BLACK				0x0000
#define WHITE				0xFFFF



void SPI_init(void);
void LCD_reset(void);
void transferComm(uint8_t send_data);
void transferData(uint8_t send_data);
void LCD_init(void);
void LCD_Fill_Color(uint16_t color);
// Drawing functions
void drawBackground(uint8_t color1, uint8_t color2);

void drawRectangle(uint16_t H_start, uint16_t H_end, uint16_t V_start, uint16_t V_end, uint8_t color1, uint8_t color2);

void drawBetterBar(uint16_t bar_x, uint16_t bar_y, uint8_t color1, uint8_t color2);

void eraseBar(uint16_t bar_x, uint16_t bar_y);

void dma_write_data(uint8_t *buff, size_t buff_size);

void LCD_SetAddressWindow(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);

void drawRectangleDMA(uint16_t xStart, uint16_t xEnd, uint16_t yStart, uint16_t yEnd, uint16_t color);

void updateBarGraph(int32_t *barHeights);

void LCD_WriteChar1206(uint16_t x, uint16_t y, char c, uint16_t fgColor, uint16_t bgColor);

void LCD_WriteString1206(uint16_t x, uint16_t y, const char *str, uint16_t fgColor, uint16_t bgColor) ;

void LCD_WriteStringAtBar(uint8_t barIndex, uint16_t y, const char *str, uint16_t fgColor, uint16_t bgColor);
#endif /* INC_LCD_H_ */
