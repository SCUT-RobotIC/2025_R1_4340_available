#ifndef R1_DRIBBLE_H
#define R1_DRIBBLE_H

#include "main.h"
#include "fdcan.h"
#include "tim.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "gpio.h"

#define SOLENOID_VALVE_TIME 200 //				电磁阀开和闭之间的延时，不重要
#define TOTAL_WAIT_TIME 500 //						放爪子到合爪子的总时间，待测
#define CIRCLES_OPEN 16 // 								开爪子圈数
#define LOAD_POSITION 8191*10 //					能把球放到发射机构上的位置，待测
#define CIRCLES_UP 10 //									导轨上到顶的圈数
#define LOAD_ANGLE -8191*10 //						上弹时爪子往下翻的角度，待测

void Dribble_test( int delay_time );

void loading(void);//上弹

#endif
