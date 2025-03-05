/*
 * lcd.h
 *
 *  Created on: Feb 27, 2025
 *      Author: Michael Rosol
 */
//
//#include "stm32f4xx_hal.h"
//#include <string.h>
//
//#ifndef INC_LCDBACKUP_H_
//#define INC_LCDBACKUP_H_
//
//
//#define LCD_ROWS 2
//#define LCD_COLS 16
//
//#define LCD_RS HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET)
//#define LCD_RW HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET)
//#define LCD_ENABLE HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
//
//// Digital Pins
//#define LCD_D0 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
//#define LCD_D1 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
//#define LCD_D2 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
//#define LCD_D3 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)
//#define LCD_D4 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)
//#define LCD_D5 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET)
//#define LCD_D6 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
//#define LCD_D7 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET)
//
//
//// Status Registers
//#define LCD_CLEAR 0x01
//#define LCD_RETURN_HOME 	0x02
//
//
//#define LCD_ENTRY_MODE_SET 	0x03
//#define LCD_CURSOR_INC 		0x01
//#define LCD_CURSOR_DEC 		0x00
//#define LCD_CURSOR_SHIFT	0x01 // shift right
//
//
//
//
//#define LCD_DISPLAY			0x08
//#define LCD_DISPLAY_ON 		0x04
//#define LCD_CURSOR_ON		0x02
//#define LCD_BLINK_ON 		0x01
//
//// may not need
//#define LCD_CURSORSHIFT 	0x10
//
//
//#define LCD_FUNCTIONSET		0x20
//#define OPT_DL 0x10						// Set interface data length
//#define OPT_N 0x08						// Set number of display lines
//#define OPT_F 0x04						// Set alternate font
//
//
//#define LCD_SETCGRAMADDR    0x40
//#define LCD_SETDDRAMADDR    0x80
//
//#define LCD_NIB 4
//#define LCD_BYTE 8
//#define LCD_DATA_REG 1
//#define LCD_COMMAND_REG 0
//
//
//// ===== Functions ===== //
//void initLCD(void);
//void clearLCD(void);
//void putLCD(char c);
//void writeLCD(char *str);
//void setCursor(char x,char y);
//void cursorOn(void);
//void blinkOn(void);
//
///**
//  * @brief Clears the cursor and the blink bits.
//  */
//void clearDisp(void);
//
///**
//  * @brief To set or reset the display/cursor/blink bits.
//  * @param dispSetting Use or(|) operation to set multiple bit.
//  */
//void setDisplay(lcdDispSetting_t dispSetting);
//
//


#endif /* INC_LCD_H_ */
