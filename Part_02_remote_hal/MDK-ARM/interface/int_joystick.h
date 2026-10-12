#ifndef INT_JOYSTICK_H
#define INT_JOYSTICK_H

#include "adc.h"

typedef struct
{
	uint16_t thr;
	uint16_t yaw;
	uint16_t roll;
	uint16_t pitch;
}Joystick_Struct;

//初始化ADC摇杆，打开ADC
void Int_Joystick_Init(void);

//获取摇杆数据,joystick:摇杆数据结构体指针
//保存摇杆数据到joystick结构体中
void Int_Joystick_Get(Joystick_Struct *joystick);

#endif // INT_JOYSTICK_H
