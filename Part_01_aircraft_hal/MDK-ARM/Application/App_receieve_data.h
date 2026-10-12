#ifndef _APP_RECEIEVE_DATA_H_
#define _APP_RECEIEVE_DATA_H_

#include "int_SI24R1.h"
#include "Com_config.h"
#include "int_VL53L1.h"


//定义帧头校验的值
#define FRAME_HEAD_CHECK_1 's'
#define FRAME_HEAD_CHECK_2 'y'
#define FRAME_HEAD_CHECK_3 'h'

//最大重试次数
#define MAX_RETRY_COUNT 10

//接收遥控器发送的数据，解析为结构体
//返回值：0-校验成功，1-校验失败/未接收到数据
uint8_t App_receieve_data(void);

//处理连接状态
//res:上一次接收数据的返回值
void App_process_connect_state(uint8_t res);

//处理飞行状态
void App_process_flight_state(void);

#endif // _APP_RECEIEVE_DATA_H_
