#include "r1_shoot.h"
#include "main.h"
#include "bsp_fdcan.h"

extern TIM_HandleTypeDef htim4;
extern motor_measure_t *motor_data_can1[8];



shoot_t shoot_data = {0, 0, 0, 0, 0, 0, 1000, 85, 70};

// Let the shoot_ctl=1, reload
// Let the shoot_ctl=4, shoot

void reload() { // Put the function into a 1khz timer!
	if (shoot_data.shoot_ctl != 0) shoot_data.cnt0++;
	else return;

	if (shoot_data.shoot_ctl == 1 && shoot_data.cnt0 >= 100) {
		shoot_data.cnt0 = 0;
		if (shoot_data.cnt1 >= 10) {
			shoot_data.cnt1 = 0;
			shoot_data.output_stg = 0;
			shoot_data.shoot_ctl = 2;
		} else {
			shoot_data.circle_d = motor_data_can1[0]->circle - shoot_data.circle_last;
			shoot_data.circle_last = motor_data_can1[0]->circle;
			if (shoot_data.circle_d == 0) shoot_data.cnt1++;
			shoot_data.output_stg += 300;
		}
	} else if (shoot_data.shoot_ctl == 2 && shoot_data.cnt0 >= 500) {
		shoot_data.cnt0 = 0;
		shoot_data.shoot_ctl = 3;
		shoot_data.output_stg = -800;
	} else if (shoot_data.shoot_ctl == 3 && shoot_data.cnt0 >= 3000) {
		shoot_data.cnt0 = 0;
		shoot_data.shoot_ctl = 0;
		shoot_data.output_stg = 0;
	} else if (shoot_data.shoot_ctl == 4) {
		shoot_data.cnt0 = 0;
		shoot_data.shoot_ctl = 5;
		shoot_data.output_servo0 = 115;
		shoot_data.output_servo1 = 40;
		SetServo(&htim4, TIM_CHANNEL_1, shoot_data.output_servo0, 20000, 20);
		SetServo(&htim4, TIM_CHANNEL_2, shoot_data.output_servo1, 20000, 20);
	} else if (shoot_data.shoot_ctl == 5 && shoot_data.cnt0 >= 1000) {
		shoot_data.cnt0 = 0;
		shoot_data.shoot_ctl = 0;
		shoot_data.output_servo0 = 85;
		shoot_data.output_servo1 = 70;
		SetServo(&htim4, TIM_CHANNEL_1, shoot_data.output_servo0, 20000, 20);
		SetServo(&htim4, TIM_CHANNEL_2, shoot_data.output_servo1, 20000, 20);
	}
}

//Digital isolation PWM, positive output
void SetServo(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t angle, uint32_t countPeriod, uint32_t CycleTime) {
	uint16_t compare_value = 0;
	if (angle <= 180) {
		compare_value = 0.5 * countPeriod / CycleTime + angle * countPeriod / CycleTime / 90;
		__HAL_TIM_SET_COMPARE(htim, Channel, compare_value);
	}
}

// Opto-isolated PWM, inverse output
void SetServoInv(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t angle, uint32_t countPeriod, uint32_t CycleTime) {
	uint16_t compare_value = 0;
	if (angle <= 180) {
		compare_value = 20000 - (0.5 * countPeriod / CycleTime + angle * countPeriod / CycleTime / 90);
		__HAL_TIM_SET_COMPARE(htim, Channel, compare_value);
	}
}
