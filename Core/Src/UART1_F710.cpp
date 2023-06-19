// SBDBTからUART通信で受け取ったコントローラのデータを格納
#include "main.h"
#include "stm32f1xx_hal_uart.h"
#include "UART1_F710.hpp"

extern UART_HandleTypeDef huart2;

Controller controller;

// UART通信の受信完了時に、ボタンのON/OFFやアナログスティックの値を代入
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	controller.B     = (controller.RxBuffer[2] & 0b01000000) >> 6;
	controller.A     = (controller.RxBuffer[2] & 0b00100000) >> 5;
	controller.X     = (controller.RxBuffer[1] & 0b00000001);
	controller.Y     = (controller.RxBuffer[2] & 0b00010000) >> 4;

	controller.RIGHT = (controller.RxBuffer[2] & 0b00000100) >> 2;
	controller.DOWN  = (controller.RxBuffer[2] & 0b00000010) >> 1;
	controller.LEFT  = (controller.RxBuffer[2] & 0b00001000) >> 3;
	controller.UP    = (controller.RxBuffer[2] & 0b00000001);

	controller.R1    = (controller.RxBuffer[1] & 0b00001000) >> 3;
	controller.R2    = (controller.RxBuffer[1] & 0b00010000) >> 4;
	controller.L1    = (controller.RxBuffer[1] & 0b00000010) >> 1;
	controller.L2    = (controller.RxBuffer[1] & 0b00000100) >> 2;

	controller.START = (controller.RxBuffer[1] & 0b00100000) >> 5;
	controller.BACK  = (controller.RxBuffer[1] & 0b01000000) >> 6;

	controller.RightAxisX = controller.RxBuffer[5];
	controller.RightAxisY = controller.RxBuffer[6];
	controller.LeftAxisX  = controller.RxBuffer[3];
	controller.LeftAxisY  = controller.RxBuffer[4];

	HAL_UART_Receive_IT(&huart2, controller.RxBuffer, 8);
}
