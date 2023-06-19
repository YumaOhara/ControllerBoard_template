#include <main.h>
#include "stm32f1xx_hal_uart.h"		// SBDBTとUART通信をするためのライブラリ
#include "UART1_F710.hpp"

// UART用構造体変数
extern UART_HandleTypeDef huart2;

// タイマ用構造体変数(適宜コメントアウトを外してタイマを有効化する)
//extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim2;
//extern TIM_HandleTypeDef htim3;
//extern TIM_HandleTypeDef htim4;

//ボタンについて
extern Controller controller;

// メイン関数
void main_cpp(void)
{
	// UART開始
	HAL_UART_Receive_IT(&huart2, controller.RxBuffer, 8);

	// LED緑を点灯
	HAL_GPIO_WritePin(GPIOC, GREEN_LED_Pin, GPIO_PIN_SET);

	// 無効化後、黄色LEDを消灯、赤色LEDを点灯
	HAL_GPIO_WritePin(GPIOC, YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, RED_LED_Pin, GPIO_PIN_SET);

	// メインループ
	while(true)
	{
		// もし、BACKボタンが押されている(BACK == 1)なら、
		if(controller.BACK == 1)
		{

			// 無効化後、黄色LEDを消灯、赤色LEDを点灯
			HAL_GPIO_WritePin(GPIOC, YELLOW_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, RED_LED_Pin, GPIO_PIN_SET);

			// LED緑を点灯
			HAL_GPIO_WritePin(GPIOC, GREEN_LED_Pin, GPIO_PIN_SET);

			//PWM無効化
		}

		// もし、STARTボタンが押されている(START == 1)なら、
		if(controller.START == 1)
		{
			// 有効化後、黄色LEDを点灯、赤色LEDを消灯
			HAL_GPIO_WritePin(GPIOC, YELLOW_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, RED_LED_Pin, GPIO_PIN_RESET);

			// LED緑を点灯
			HAL_GPIO_WritePin(GPIOC, GREEN_LED_Pin, GPIO_PIN_SET);

			//PWM有効化

		}
	}
}
