#include <main.h>
#include <usart.h>
#include <tim.h>
#include "stm32f1xx_hal_uart.h"
#include "UART1_F710.hpp"
#include <math.h>

//PA1 CH1 射出
//PA8 CH1 前方
//PA9 CH2 左
//PA10 CH3 右

// キャリブレーション状態を管理するフラグ
bool calibration_done = false;

const uint16_t PWM_MIN = 1000;
const uint16_t PWM_MAX = 2000;

// CH1,2,3 (双方向オムニ用)
const uint16_t OMNI_PWM_NEUTRAL = 1500;
const float MAX_OMNI_SPEED = 1000.0f; // 内部計算用の最大速度値

// 回転速度の固定値 (MAX_OMNI_SPEED=1000の50%を使用)
const float ROTATION_SPEED_Vz = 0.5f; // 正規化された回転速度 (50%のトルク/速度を指令)

// コントローラー入力設定 (0〜127)
const int16_t JOY_CENTER = 64;
const float JOY_MAX_DEVIATION = 63.0f;
const int16_t JOY_DEAD_ZONE = 5;


float NormalizeJoystick(int16_t joy_input)
{
    if (abs(joy_input - JOY_CENTER) <= JOY_DEAD_ZONE) return 0.0f;
    float normalized = (float)(joy_input - JOY_CENTER) / JOY_MAX_DEVIATION;
    if (normalized > 1.0f) return 1.0f;
    if (normalized < -1.0f) return -1.0f;
    return normalized;
}

int16_t Motor_M2_Cmd = 0;
int16_t Motor_M3_Cmd = 0;
int16_t Motor_M4_Cmd = 0;


uint16_t SpeedToBiDirPWM(int16_t speed_cmd)
{
    float normalized_speed = (float)speed_cmd / MAX_OMNI_SPEED;
    int32_t pulse_width = (int32_t)(OMNI_PWM_NEUTRAL + normalized_speed * 500.0f);
    if (pulse_width > PWM_MAX) return PWM_MAX;
    if (pulse_width < PWM_MIN) return PWM_MIN;
    return (uint16_t)pulse_width;
}

void CalculateOmniSpeeds(float Vx, float Vy, float Vz)
{
    // モーター配置角度 (ラジアン) M2=0°, M3=120°, M4=240°
    const float M2_ANGLE = 0.0f;
    const float M3_ANGLE = 2.094395f;
    const float M4_ANGLE = 4.188790f;

    // 各モーターの速度ベクトル計算 (MAX_OMNI_SPEEDを乗算して内部スケールに戻す)
    float m2 = (Vx * cosf(M2_ANGLE) + Vy * sinf(M2_ANGLE) + Vz) * MAX_OMNI_SPEED;
    float m3 = (Vx * cosf(M3_ANGLE) + Vy * sinf(M3_ANGLE) + Vz) * MAX_OMNI_SPEED;
    float m4 = (Vx * cosf(M4_ANGLE) + Vy * sinf(M4_ANGLE) + Vz) * MAX_OMNI_SPEED;

    // 最大速度を超えないようにスケーリング
    float max_val = fmaxf(fabsf(m2), fmaxf(fabsf(m3), fabsf(m4)));

    if (max_val > MAX_OMNI_SPEED) {
        float scale = MAX_OMNI_SPEED / max_val;
        m2 *= scale;
        m3 *= scale;
        m4 *= scale;
    }

    Motor_M2_Cmd = (int16_t)m2;
    Motor_M3_Cmd = (int16_t)m3;
    Motor_M4_Cmd = (int16_t)m4;
}

extern "C" void main_cpp()
{
	// UART開始
	HAL_UART_Receive_IT(&huart2, controller.RxBuffer, 8);

	// LED初期設定
	HAL_GPIO_WritePin(GPIOC, GREEN_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, RED_LED_Pin, GPIO_PIN_SET);

	// 1. STM32起動時: PWMを最大値 (2000us) に設定 (キャリブレーション準備)
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 2000);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    // この状態でESCに電源を入れてください。

	// メインループ
	while(true)
	{
        if (!calibration_done)
        {
            // キャリブレーションモード中

            // LEDを調整中であることを示す (例: 緑点灯 + 黄色点滅)
            HAL_GPIO_TogglePin(GPIOC, YELLOW_LED_Pin);
            HAL_Delay(100);

            // Startボタンが押されたら、最小値に切り替えてキャリブレーションを完了
            if (controller.Start == 1)
            {
                // 3秒ルール：最小パルス幅 (1000us) に設定
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
                HAL_Delay(1000); // ESCが処理するのを待つ

                calibration_done = true;

                // キャリブレーション成功の確認（緑点滅）
                HAL_GPIO_WritePin(GPIOC, RED_LED_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOC, YELLOW_LED_Pin, GPIO_PIN_SET);
            }
        }
        else
        {
            // --- 通常制御モード ---

            // BackボタンによるPWM停止
            if(controller.Back == 1)
            {
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
                HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
                HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000);
                HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000);
                HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
                // LEDを停止状態に
                HAL_GPIO_WritePin(GPIOC, YELLOW_LED_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOC, RED_LED_Pin, GPIO_PIN_SET);
            }

            // StartボタンによるPWM再開 (キャリブレーションフラグはすでにtrueなので再キャリブレーションはしない)
            if(controller.Start == 1)
            {
                // 停止中の場合のみ再開
                HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
                // LEDを有効状態に
                HAL_GPIO_WritePin(GPIOC, YELLOW_LED_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOC, RED_LED_Pin, GPIO_PIN_RESET);
            }

            if(controller.B == 1){
				// 加速パラメータ定義
				const uint16_t START_PULSE = 1000;
				const uint16_t MAX_PULSE = 2000;
				const uint16_t STEP_SIZE = 20;  // 1回の加速で増やすパルス幅 (us)
				const uint32_t DELAY_MS = 30;   // 加速ステップ間の遅延 (ms)

				uint16_t current_pulse;

				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, START_PULSE);

				// 加速ループ
				for (current_pulse = START_PULSE + STEP_SIZE; current_pulse <= MAX_PULSE; current_pulse += STEP_SIZE)
				{
					// 2000usを超えないようにクランプ
					if (current_pulse > MAX_PULSE)
					{
						current_pulse = MAX_PULSE;
					}

					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pulse);
					HAL_Delay(DELAY_MS);
				}

				// 加速完了後、最高速を維持
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MAX_PULSE);
			}
            if(controller.Y == 1){
				// 減速パラメータ定義
				const uint16_t TARGET_PULSE = 1000;
				const uint16_t STEP_SIZE = 20;  // 1回の減速で減らすパルス幅 (us)
				const uint32_t DELAY_MS = 30;   // 減速ステップ間の遅延 (ms)

				// 現在のパルス幅を取得 (正確な値が取れない場合は、適当な開始値(2000)を設定しても良い)
				// 簡略化のため、ここでは現在の値ではなく2000からスタートすると仮定します。
				uint16_t current_pulse = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);

				// ただし、現在のパルス幅が既に1000に近い場合は2000から始めるか判定が必要
				if (current_pulse < TARGET_PULSE) {
					current_pulse = TARGET_PULSE; // 既に停止しているなら何もしない
				}

				// 2000usから1000usまで減速
				while (current_pulse > TARGET_PULSE)
				{
					// STEP_SIZE分減少させる
					if (current_pulse > (TARGET_PULSE + STEP_SIZE))
					{
						current_pulse -= STEP_SIZE;
					}
					else
					{
						// ターゲット値を超えそうになったらターゲット値に設定
						current_pulse = TARGET_PULSE;
					}

					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, current_pulse);
					HAL_Delay(DELAY_MS);
				}

				// 減速完了後、停止値(1000us)を維持
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, TARGET_PULSE);
            }
            if(controller.X == 1){
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1700);
            }
            if(controller.A == 1){
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
            }

            float Vx = NormalizeJoystick(controller.LeftAxisX); // 左右移動
			float Vy = -NormalizeJoystick(controller.LeftAxisY); // 前後移動

			// 2. 回転速度
			float Vz = 0.0f;
			if (controller.R1 == 1) {
				// R1: 右回転 (正方向/時計回り, Vz +)
				Vz = ROTATION_SPEED_Vz;
			} else if (controller.L1 == 1) {
				// L1: 左回転 (逆方向/反時計回り, Vz -)
				Vz = -ROTATION_SPEED_Vz;
			}

			// 速度計算
			CalculateOmniSpeeds(Vx, Vy, Vz);

			// PWM出力 (双方向モード)
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, SpeedToBiDirPWM(Motor_M2_Cmd));
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, SpeedToBiDirPWM(Motor_M3_Cmd));
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, SpeedToBiDirPWM(Motor_M4_Cmd));
        }
	}
}
