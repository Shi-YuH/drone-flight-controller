#ifndef INT_KEY_H
#define INT_KEY_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

typedef enum{

    KEY_NONE = 0,
    KEY_UP,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_LEFT_X,
    KEY_RIGHT_X,
    KEY_RIGHT_X_LONG,
}KEY_type;

// @brief:获取按键是否被按下
KEY_type Int_KEY_Get(void);

#endif // __INT_KEY_H__
