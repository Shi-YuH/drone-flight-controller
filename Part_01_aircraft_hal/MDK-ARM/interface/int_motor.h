#ifndef INT_MOTOR_H
#define INT_MOTOR_H

#include "tim.h"
#include "Com_debug.h"

typedef struct
{
    TIM_HandleTypeDef *tim;
    uint16_t channel;
    int16_t speed;

}MOTOR_STRUCT;

void int_motor_set_speed(MOTOR_STRUCT *motor);

void int_motor_start(MOTOR_STRUCT *motor);

#endif
