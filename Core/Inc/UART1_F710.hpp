#pragma once
#include <cstdint>
using std::uint8_t;

struct Controller {
	uint8_t RxBuffer[8] = {};
	volatile uint8_t B = 0;
	volatile uint8_t A = 0;
	volatile uint8_t X = 0;
	volatile uint8_t Y = 0;

	volatile uint8_t Right = 0;
	volatile uint8_t Down = 0;
	volatile uint8_t Left = 0;
	volatile uint8_t Up = 0;

	volatile uint8_t R1 = 0;
	volatile uint8_t R2 = 0;
	volatile uint8_t L1 = 0;
	volatile uint8_t L2 = 0;

	volatile uint8_t Start = 0;
	volatile uint8_t Back  = 0;

	volatile uint8_t RightAxisX = 64;
	volatile uint8_t RightAxisY = 64;
	volatile uint8_t LeftAxisX  = 64;
	volatile uint8_t LeftAxisY  = 64;
};

//ボタンについて
extern Controller controller;
