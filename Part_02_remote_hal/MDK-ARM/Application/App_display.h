#ifndef __APP_DISPLAY_H__
#define __APP_DISPLAY_H__

#include "int_oled.h"
#include "int_SI24R1.h"
#include "App_process_data.h"

#define LINE1_BEGIN 28
#define LINE2_BEGIN 5
#define LINE3_BEGIN 5
#define LINE3_BEGIN2 65
#define LINE4_BEGIN 5
#define LINE4_BEGIN2 65

#define BAR1_BEGIN 35
#define BAR2_BEGIN 47
#define BAR1_BEGIN2 95
#define BAR2_BEGIN2 107


#define Y0 0
#define Y1 14
#define Y2 26
#define Y3 38



//初始化
void App_Display_Init(void);

//显示数据
void App_Display_Show(void);

#endif
