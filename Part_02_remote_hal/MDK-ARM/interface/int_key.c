#include "int_key.h"

KEY_type Int_KEY_Get(void)
{
    
    if(HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin) == GPIO_PIN_RESET)
    {
        //1.按键消抖
        vTaskDelay(10);
        //key_up被按下
        if(HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin) == GPIO_PIN_RESET)
        {
            //2.避免按键时间过长，多次判断，等待按键抬起后再返回
            while(HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin) == GPIO_PIN_RESET)
            {
                vTaskDelay(1);
            }
            return KEY_UP;
        }
    }
    else if(HAL_GPIO_ReadPin(KEY_DOWN_GPIO_Port, KEY_DOWN_Pin) == GPIO_PIN_RESET)
    {
        //1.按键消抖
		vTaskDelay(10);
		//key_down被按下
		if(HAL_GPIO_ReadPin(KEY_DOWN_GPIO_Port, KEY_DOWN_Pin) == GPIO_PIN_RESET)
		{
			//2.避免按键时间过长，多次判断，等待按键抬起后再返回
			while(HAL_GPIO_ReadPin(KEY_DOWN_GPIO_Port, KEY_DOWN_Pin) == GPIO_PIN_RESET)
            {
                vTaskDelay(1);
            }
			return KEY_DOWN;
		}
    }
    else if(HAL_GPIO_ReadPin(KEY_LEFT_GPIO_Port, KEY_LEFT_Pin) == GPIO_PIN_RESET)
    {
        //1.按键消抖
        vTaskDelay(10);
        //key_left被按下
        if(HAL_GPIO_ReadPin(KEY_LEFT_GPIO_Port, KEY_LEFT_Pin) == GPIO_PIN_RESET)
        {
            //2.避免按键时间过长，多次判断，等待按键抬起后再返回
            while(HAL_GPIO_ReadPin(KEY_LEFT_GPIO_Port, KEY_LEFT_Pin) == GPIO_PIN_RESET)
            {
                vTaskDelay(1);
            }
            return KEY_LEFT;
        }
    }
    else if(HAL_GPIO_ReadPin(KEY_RIGHT_GPIO_Port, KEY_RIGHT_Pin) == GPIO_PIN_RESET)
    {
        //1.按键消抖
        vTaskDelay(10);
        //key_right被按下
        if(HAL_GPIO_ReadPin(KEY_RIGHT_GPIO_Port, KEY_RIGHT_Pin) == GPIO_PIN_RESET)
        {
            //2.避免按键时间过长，多次判断，等待按键抬起后再返回
            while(HAL_GPIO_ReadPin(KEY_RIGHT_GPIO_Port, KEY_RIGHT_Pin) == GPIO_PIN_RESET)
            {
                vTaskDelay(1);
            }
            return KEY_RIGHT;
        }
    }
    else if(HAL_GPIO_ReadPin(KEY_LEFT_X_GPIO_Port, KEY_LEFT_X_Pin) == GPIO_PIN_RESET)
    {
        //1.按键消抖
        vTaskDelay(10);
        //key_left_x被按下
        if(HAL_GPIO_ReadPin(KEY_LEFT_X_GPIO_Port, KEY_LEFT_X_Pin) == GPIO_PIN_RESET)
        {
            //2.避免按键时间过长，多次判断，等待按键抬起后再返回
            while(HAL_GPIO_ReadPin(KEY_LEFT_X_GPIO_Port, KEY_LEFT_X_Pin) == GPIO_PIN_RESET)
            {
                vTaskDelay(1);
            }
            return KEY_LEFT_X;
        }
    }

    //右上按键
    else if(HAL_GPIO_ReadPin(KEY_RIGHT_X_GPIO_Port, KEY_RIGHT_X_Pin) == GPIO_PIN_RESET)
    {
        //1.此时开始计时，长按为超过1s
        TickType_t count1 = xTaskGetTickCount();
        //按键消抖
        vTaskDelay(10);
        //key_right_x被按下
        if(HAL_GPIO_ReadPin(KEY_RIGHT_X_GPIO_Port, KEY_RIGHT_X_Pin) == GPIO_PIN_RESET)
        {
            //2.避免按键时间过长，多次判断，等待按键抬起后再返回
            while(HAL_GPIO_ReadPin(KEY_RIGHT_X_GPIO_Port, KEY_RIGHT_X_Pin) == GPIO_PIN_RESET)
            {
                vTaskDelay(1);
            }
            TickType_t count2 = xTaskGetTickCount();
            if(count2 - count1 > pdMS_TO_TICKS(1000))
            {
                return KEY_RIGHT_X_LONG;
            }
            else
            {
                return KEY_RIGHT_X;
            }
        }
    }
    return KEY_NONE;
}

