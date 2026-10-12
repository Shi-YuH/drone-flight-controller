#ifndef COM_CONFIG_H
#define COM_CONFIG_H

#include "main.h"

//链接状态
typedef enum
{
    REMOTE_STATE_CONNECTED = 0,
    REMOTE_STATE_DISCONNECTED 
}Remote_State;
//飞行状态
typedef enum
{
    FLIGHT_STATE_IDLE = 0,
    FLIGHT_STATE_NORMAL,
    FLIGHT_STATE_FIX_HEIGHT,
    FLIGHT_STATE_FAIL
}Flight_State;

//油门解锁状态
typedef enum
{
    FREE = 0,
    MAX,
    LEAVE_MAX,
    MIN,
    UNLOCK
}Thr_State;

//遥控数据
typedef struct{
    int16_t thr;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    uint8_t shutdown;//关机按键,1:关机，0:不关机
    uint8_t fix_height;//固定高度按键,1:切换固定高度，0:不切换
}Remote_Data;
//陀螺仪数据
typedef struct{
    int16_t gyro_x;//往右飞转动为+ ，表示翻滚角
    int16_t gyro_y;//向前飞转动为+ ，表示俯仰角
    int16_t gyro_z;//逆时针旋转为+ ，表示偏航角
}Gyro_Data;

typedef struct{
    int16_t accel_x;//向前的加速度为+
    int16_t accel_y;//向左的加速度为+
    int16_t accel_z;//向上的加速度为+
}Accel_Data;

typedef struct{
    Gyro_Data gyro;
    Accel_Data accel;
}Gyro_Accel_Struct;

//解算得到的欧拉角
typedef struct{
    float roll;
    float pitch;
    float yaw;
}Euler_Struct;
#endif 
