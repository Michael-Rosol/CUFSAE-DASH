#ifndef DISPLAY_H
#define DISPLAY_H


#include "main.h"


#define D1_HIGH() HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET)
#define D1_LOW() HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET)
#define D2_HIGH() HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET)
#define D2_LOW() HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET)
#define D3_HIGH() HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET)
#define D3_LOW() HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET)
#define D4_HIGH() HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET)
#define D4_LOW() HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET)
#define DP_ON() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)  // Turn on decimal point
#define DP_OFF() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)  // Turn off decimal point



void SevenSegment_Update(uint8_t number);
void DisplayRxData(uint32_t value);


#endif

