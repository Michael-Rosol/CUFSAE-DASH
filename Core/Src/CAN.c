
#include "CAN.h"

Rx rx_msg;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // Retrieve the message contents
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_msg.Header, rx_msg.Data) != HAL_OK)
    {
        Error_Handler();
    }
}

void CANInterpret(Rx rx_msg)
{
    // Do whatever processing you need with the received data

	// Getting the Battery Voltage
	if (rx_msg.Header.StdId == 0x660){

		uint8_t raw_voltage = rx_msg.Data[5];
		uint8_t raw_gear = rx_msg.Data[4];

	}
	// Getting the Oil Temperature
	if (rx_msg.Header.StdId == 0x661) {

	}
}
