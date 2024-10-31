#include "UART.h"
#include <stdio.h>

UART_HandleTypeDef huart2;

void uart_init(void)
{
	GPIO_InitTypeDef GPIO_Init_Struct={0};


	//Enable clock access for GPIO A (PA2 & PA3)
	__HAL_RCC_GPIOA_CLK_ENABLE();
	//Enable UART module clock access
	__HAL_RCC_USART2_CLK_ENABLE();
	//Configure pins to act as alternate funcs pins (UART)
	GPIO_Init_Struct.Pin=GPIO_PIN_2|GPIO_PIN_3;
	GPIO_Init_Struct.Mode=GPIO_MODE_AF_PP;
	GPIO_Init_Struct.Alternate=GPIO_AF7_USART2;
	GPIO_Init_Struct.Pull=GPIO_NOPULL; //No Pull-up or Pull-down activation
	GPIO_Init_Struct.Speed=GPIO_SPEED_FREQ_VERY_HIGH;

	HAL_GPIO_Init(GPIOA,&GPIO_Init_Struct);

	//configure UART

	huart2.Instance=USART2;
	huart2.Init.BaudRate=250000;
	huart2.Init.WordLength=UART_WORDLENGTH_8B;
	huart2.Init.StopBits=UART_STOPBITS_1;
	huart2.Init.Parity=UART_PARITY_NONE;
	huart2.Init.Mode=UART_MODE_TX;
	huart2.Init.HwFlowCtl=UART_HWCONTROL_NONE;
	huart2.Init.OverSampling=UART_OVERSAMPLING_16;

	HAL_UART_Init(&huart2);

}
/**
 * Re-targets printf
 * !!!DO NOT TOUCH!!!
 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
	return ch;

}
