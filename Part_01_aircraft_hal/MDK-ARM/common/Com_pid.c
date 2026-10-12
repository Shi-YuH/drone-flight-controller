#include "Com_pid.h"

//单次PID计算
void Common_PID_Calc(PID_Struct *pid)
{
    //1。目标值测量 计算误差
    pid->err = pid->measure - pid->desired;

    //2.计算积分误差
    pid->integral += pid->err;

    //3.计算微分误差
    if(pid->err_last == 0)
    {
        pid->err_last = pid->err;
    }
    float diff= (pid->err - pid->err_last);

    //4.计算PID输出
    pid->output = pid->kp * pid->err + (pid->ki * pid->integral) * PID_PERIOD+ (pid->kd * diff) / PID_PERIOD;

    //5.保存误差
    pid->err_last = pid->err;
}

//串级PID计算
void Common_PID_Calc_Series(PID_Struct *out_pid,PID_Struct *in_pid)
{
    //1.先计算外环
    Common_PID_Calc(out_pid);
    //2.再计算内环 将外环的输出值作为内环的目标值
    in_pid->desired = out_pid->output;
    Common_PID_Calc(in_pid);
    
}

//电机值限幅
int16_t Com_limit(int16_t value,int16_t min,int16_t max)
{
   if(value > max)
    {
        return max;
    }
    else if(value < min)
    {
        return min;
    }
    else
    {
        return value;
    }
}
