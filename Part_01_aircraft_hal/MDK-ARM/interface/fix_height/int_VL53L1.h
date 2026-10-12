#ifndef _INT_VL53L1_H_
#define _INT_VL53L1_H_

#include "vl53l1_platform.h"
#include "vl53l1x_api.h"
#include "vl53l1x_calibration.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Com_debug.h"


//初始化VL53L1传感器
void int_VL53L1_init(void);

//获取距离
//返回值: 距离(mm)
uint16_t int_VL53L1_get_distance(void);


#endif
