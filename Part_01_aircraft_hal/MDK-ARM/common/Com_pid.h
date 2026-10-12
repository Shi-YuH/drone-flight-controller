#ifndef _COM_PID_H_
#define _COM_PID_H_

#include "main.h"

#define PID_PERIOD 0.006

//kp ki kd 需要在初始化时设置 目标值和测量值 在计算时传递
typedef struct
{
    float kp;//比例系数 值越大响应速度越快
    float ki;//积分系数 解决稳态误差 无人机控制过程中 积分项会产生积分饱和 导致控制不稳定 
    float kd;//微分系数 值越大抑制效果越好 解决超调震荡
    float err;//误差
    float err_last;//上一次误差
    float desired;//期望值
    float measure;//测量值
    float integral;//积分累计值
    float output;//输出值

} PID_Struct;

//单次PID计算
void Common_PID_Calc(PID_Struct *pid);

//串级PID计算
void Common_PID_Calc_Series(PID_Struct *out_pid,PID_Struct *in_pid);

//电机值限幅
int16_t Com_limit(int16_t value,int16_t min,int16_t max);


#endif
