#include "int_motor.h"

/* @brief:参数为比较值，设置电机速度，最大为1000，默认值为200

*/

void int_motor_set_speed(MOTOR_STRUCT *motor)
{
    if(motor->speed > 1000)
    {
        debug_printf("motor speed is too high, max is 1000\r\n");
        return;

    }
    __HAL_TIM_SET_COMPARE(motor->tim, motor->channel, motor->speed);
}

//启动电机
void int_motor_start(MOTOR_STRUCT *motor)
{
    __HAL_TIM_SET_COMPARE(motor->tim, motor->channel,0);
    HAL_TIM_PWM_Start(motor->tim, motor->channel);
}
