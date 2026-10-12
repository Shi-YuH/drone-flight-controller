#ifndef APP_TRANSMIT_DATA_H
#define APP_TRANSMIT_DATA_H

#include "int_SI24R1.h"
#include "App_process_data.h"
#include "FreeRTOS.h"
#include "task.h"

//定义帧头校验的值
#define FRAME_HEAD_CHECK_1 's'
#define FRAME_HEAD_CHECK_2 'y'
#define FRAME_HEAD_CHECK_3 'h'



//发送数据函数,自动切换si24r1的模式，将采集到的遥控数据打包发送到飞机
void App_Transmit_Data(void);

#endif // __APP_TRANSMIT_DATA_H__
