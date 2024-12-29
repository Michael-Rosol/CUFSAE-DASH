#include "display.h"





static uint8_t segmentNumber[10] = {
        0x3f,  // 0
        0x06,  // 1
        0x5b,  // 2
        0x4f,  // 3
        0x66,  // 4
        0x6d,  // 5
        0x7d,  // 6
        0x07,  // 7
        0x7f,  // 8
        0x67   // 9
};

void SevenSegment_Update(uint8_t number){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, ((number>>0)&0x01)); // a
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, ((number>>1)&0x01)); // b
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, ((number>>2)&0x01)); // c
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, ((number>>3)&0x01)); // d
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, ((number>>4)&0x01)); // e
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, ((number>>5)&0x01)); // f
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, ((number>>6)&0x01)); // g

}


void DisplayRxData(uint32_t value) {
    // Extract individual digits
    uint8_t temp1 = (value / 1000) % 10; // Thousands place
    uint8_t temp2 = (value / 100) % 10;  // Hundreds place
    uint8_t temp3 = (value / 10) % 10;   // Tens place
    uint8_t temp4 = value % 10;          // Units place

        // Display thousands digit
        SevenSegment_Update(segmentNumber[temp1]);
        D1_LOW();  // Activate D1
        HAL_Delay(7); // Short delay for this digit
        D1_HIGH(); // Deactivate D1


        // Display hundreds digit
        SevenSegment_Update(segmentNumber[temp2]);
        D2_LOW();  // Activate D2
        HAL_Delay(7); // Short delay for this digit
        D2_HIGH(); // Deactivate D2

        // Display tens digit
        SevenSegment_Update(segmentNumber[temp3]);
        D3_LOW();  // Activate D3
        HAL_Delay(7); // Short delay for this digit
        D3_HIGH(); // Deactivate D3

        // Display units digit
        SevenSegment_Update(segmentNumber[temp4]);
        D4_LOW();  // Activate D4
        HAL_Delay(7); // Short delay for this digit
        D4_HIGH(); // Deactivate D4



}



