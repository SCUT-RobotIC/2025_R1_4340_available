#include "R1_dribble.h"
#include "motorctrl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "gpio.h"

extern motor_measure_t *motor_data_can1[8];
extern motor_measure_t *motor_data_can2[8];
extern motor_measure_t *motor_data_can3[8];
uint8_t DribblingStatus=0;
void Stop(void){
		rtU.status_CH3_7=2;
		set_target(3,7,8191*CIRCLES_UP);
}
void going_up(void){

    while(!(motor_data_can3[6]->circle>CIRCLES_UP)) 
		{
				rtU.status_CH3_7=1;
        set_target(3,7,2000);
		};
	
        
}

void dribbling(int delay_time_stike,int delay_time_close_claw){
				rtU.status_CH3_7=2;
        set_target(3,5,-8191*CIRCLES_OPEN); //开爪，圈数待测
        set_target(3,7,8191); //降，圈数待测
        HAL_GPIO_WritePin(SOLENOID_VALVE_GPIO_Port, SOLENOID_VALVE_Pin, GPIO_PIN_RESET);
				osDelay(delay_time_stike);
        HAL_GPIO_WritePin(SOLENOID_VALVE_GPIO_Port, SOLENOID_VALVE_Pin, GPIO_PIN_SET);
				osDelay(delay_time_close_claw);
        set_target(3,5,0); //合爪
    // }else{
    //     loading();
//    }
}
void Dribble_test( int delay_time ){
	if(DribblingStatus==1){
		going_up();
		if(motor_data_can3[6]->circle>CIRCLES_UP)
			DribblingStatus=1;
	}
	if(DribblingStatus==2)
	{
		Stop();
		DribblingStatus=3;
	}
	if(DribblingStatus==3)
	{
		osDelay(delay_time);
		DribblingStatus=4;
	}
	if(DribblingStatus==4)
	{
		dribbling(SOLENOID_VALVE_TIME,TOTAL_WAIT_TIME-SOLENOID_VALVE_TIME);
		DribblingStatus=0;
	}
	
}

void loading(void){
    set_target(3,6,LOAD_POSITION);//升降至合适高度
    set_target(3,7,LOAD_ANGLE);//爪子下翻递球
    osDelay(500);
    set_target(1,6,8191*CIRCLES_UP);//升降至最高位，不影响投球
    set_target(3,7,0);//爪子归位
}
