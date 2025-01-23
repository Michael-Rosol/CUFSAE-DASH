
#include "CAN.h"
#include <stdio.h>
extern CAN_HandleTypeDef hcan2;
// global variables for passing the argument to main
Rx rx_msg;

int voltage = 0; //change to 0 later as default
uint16_t raw_rpm = 0;
uint8_t raw_voltage;
uint8_t raw_gear = 0;
volatile uint16_t oil_temp = 0;

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//    // Retrieve the message contents
//    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_msg.Header, rx_msg.Data) != HAL_OK)
//    {
////    	CANInterpret(&rx_msg);
//    	Error_Handler();
//
//    } else {
//    	Error_Handler();
//    }
//}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	if (hcan->Instance == CAN2) {
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_msg.Header, rx_msg.Data) != HAL_OK)
  {
	  CANInterpret(&rx_msg);
    /* Reception Error */
    Error_Handler();
  	  }
	}
}



void CANInterpret(Rx* rx_msg)
{


	// Getting the Battery Voltage
	if (rx_msg->Header.StdId == 0x660){

		raw_rpm = ((uint16_t)rx_msg->Data[1]) | (uint16_t)rx_msg->Data[0];
		raw_voltage = rx_msg->Data[5];
		raw_gear = rx_msg->Data[4];

	}
	// Getting the Oil Temperature
	 if (rx_msg->Header.StdId == 0x661) {
		 oil_temp = ((uint16_t)rx_msg->Data[1] << 8) | (uint16_t)rx_msg->Data[0];
	}

	 if (rx_msg->Header.StdId == 0x0446) {
		 raw_voltage = 60;
	}



	//voltage = (int)(raw_voltage / 10);

}

