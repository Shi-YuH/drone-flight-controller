#ifndef _APP_FLIGHT_H_
#define _APP_FLIGHT_H_

#include "int_mpu6050.h"
#include "Com_debug.h"
#include "Com_filter.h"
#include "math.h"
#include "Com_imu.h"
#include "Com_pid.h"
#include "int_motor.h"
#include "int_VL53L1.h"


//初始化 MPU6050 和 电机
void App_flight_init(void);

//根据测量的数据计算出欧拉角
void App_flight_get_euler_angle(void);

//根据欧拉角计算出目标的PID
void App_flight_pid_calc(void);

//执行PID控制
void App_flight_pid_control_motor(void);

//定高PID
void App_flight_fix_height_pid_calc(void);

#endif 
