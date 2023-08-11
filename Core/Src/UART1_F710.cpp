// SBDBTからUART通信で受け取ったコントローラのデータを格納
#include "usart.h"
#include "UART1_F710.hpp"

Controller controller;

// UART通信の受信完了時に、ボタンのON/OFFやアナログスティックの値を代入
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	controller.B     = (controller.RxBuffer[2] & 0b01000000) >> 6;
	controller.A     = (controller.RxBuffer[2] & 0b00100000) >> 5;
	controller.X     = (controller.RxBuffer[1] & 0b00000001);
	controller.Y     = (controller.RxBuffer[2] & 0b00010000) >> 4;

	controller.Right = (controller.RxBuffer[2] & 0b00000100) >> 2;
	controller.Down  = (controller.RxBuffer[2] & 0b00000010) >> 1;
	controller.Left  = (controller.RxBuffer[2] & 0b00001000) >> 3;
	controller.Up    = (controller.RxBuffer[2] & 0b00000001);

	controller.R1    = (controller.RxBuffer[1] & 0b00001000) >> 3;
	controller.R2    = (controller.RxBuffer[1] & 0b00010000) >> 4;
	controller.L1    = (controller.RxBuffer[1] & 0b00000010) >> 1;
	controller.L2    = (controller.RxBuffer[1] & 0b00000100) >> 2;

	controller.Start = (controller.RxBuffer[1] & 0b00100000) >> 5;
	controller.Back  = (controller.RxBuffer[1] & 0b01000000) >> 6;

	controller.RightAxisX = controller.RxBuffer[5];
	controller.RightAxisY = controller.RxBuffer[6];
	controller.LeftAxisX  = controller.RxBuffer[3];
	controller.LeftAxisY  = controller.RxBuffer[4];

	HAL_UART_Receive_IT(&huart2, controller.RxBuffer, 8);
}
