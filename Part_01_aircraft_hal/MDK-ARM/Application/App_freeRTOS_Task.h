#ifndef APP_FREERTOS_TASK_H
#define APP_FREERTOS_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "Com_debug.h"
#include "int_P5305T.h"
#include "int_motor.h"
#include "int_led.h"
#include "Com_config.h"
#include "int_SI24R1.h"
#include "App_receieve_data.h"
#include "App_flight.h"
#include "int_bat_adc.h"



void App_freeRTOS_start(void);

#endif //APP_FREERTOS_TASK_H
