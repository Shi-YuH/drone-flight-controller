#ifndef COM_DEBUGER_H
#define COM_DEBUGER_H

#include "stm32f1xx_hal.h"
#include "usart.h"
#include "stdio.h"
#include"stdarg.h"
#include"string.h"

//使用串口打印日志非常占用CPU资源，通过比特率计算可知，10字节占用大概1ms，影响飞机的运行，所以需要优化
//在后续飞机正常飞行时，需要关闭串口打印
//设计一个日志打印开关
#define DEBUG_ENABLE 1

#ifdef DEBUG_ENABLE

//使用宏定义的方式只打印文件名，不打印路径名
#define __FILE_NAME__ (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1):__FILE__)

//使用宏定义的方式实现打印日志之前，输出文件名和行号
#define debug_printf(format, ...) printf("[%s:%d] "    format    "\r\n", __FILE_NAME__, __LINE__, ##__VA_ARGS__)

#else
//当调试关闭时，将debug_printf宏定义为空操作
#define debug_printf(format, ...)
#endif  


#endif
