#ifndef APP_PROCESS_DATA_H
#define APP_PROCESS_DATA_H

#include "int_joystick.h"
#include "int_key.h"
#include "Com_debug.h"
#include "Com_tool.h"

//摇杆数据结构体（仅在 .c 中定义一次，头文件只做声明）
extern Joystick_Struct joystick;

//遥控数据
typedef struct{
    int16_t thr;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    uint8_t shutdown;//关机按键,1:关机，0:不关机
    uint8_t fix_height;//固定高度按键,1:切换固定高度，0:不切换
}Remote_Data;

//处理按键数据，对按下的按键进行记录
void App_Process_Key_Data(void);

//处理摇杆数据，修正极相数据和标准值
void App_Process_Joystick_Data(void);


#endif // __APP_PROCESS_DATA_H__
