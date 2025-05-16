#ifndef R1_SHOOT_H
#define R1_SHOOT_H

#include "main.h"
typedef struct shoot {
	uint16_t cnt0;
	uint16_t cnt1;
	uint8_t shoot_ctl; // state machine
	int16_t output_stg; // current output
	int16_t output_2006; // distance, angle pid
	int16_t circle_last;
	int16_t circle_d;
	uint8_t output_servo0; // 85 115
	uint8_t output_servo1; // 70 40
} shoot_t;
void reload(void);
void SetServo(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t angle, uint32_t countPeriod, uint32_t CycleTime);
void SetServoInv(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t angle, uint32_t countPeriod, uint32_t CycleTime);

#endif // R1_SHOOT_H
