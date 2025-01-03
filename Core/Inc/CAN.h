#ifndef CAN_H
#define CAN_H

#include "stm32f4xx_hal.h"
#include "main.h"

typedef struct {
	CAN_RxHeaderTypeDef Header;
	uint8_t Data[8];
} Rx;



extern Rx rx_msg;

void CANInterpret(Rx rx_msg);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);


#endif
