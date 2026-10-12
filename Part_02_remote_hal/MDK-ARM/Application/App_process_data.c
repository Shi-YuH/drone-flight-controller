#include "App_process_data.h"

Joystick_Struct joystick = {0};

Remote_Data remote_data = {0};

//区分摇杆值和按键微调值
int16_t key_pitch_offset = 0;//往前飞是+，往后飞是-
int16_t key_roll_offset = 0;//往左飞是-，往右飞是+

//摇杆零偏值
int16_t thr_offset = 0;
int16_t yaw_offset = 0;
int16_t pitch_offset = 0;
int16_t roll_offset = 0;

//校准摇杆函数
static void App_Calibrate_Joystick(void)
{
    //减去零偏的值
    //先清空按键的微调值
	key_pitch_offset = 0;
	key_roll_offset = 0;

    //多次获取一次摇杆数据，然后取平均值，作为零偏值
    int16_t thr_sum = 0;
    int16_t yaw_sum = 0;
    int16_t pitch_sum = 0;
    int16_t roll_sum = 0;

    for(int i = 0; i < 10; i++)
    {
        App_Process_Joystick_Data();
        thr_sum += joystick.thr - 0;
        yaw_sum += joystick.yaw - 500;
        pitch_sum += joystick.pitch - 500;
        roll_sum += joystick.roll - 500;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    //零偏校准没有累加，长按两次会回退
    thr_offset += thr_sum / 10;
    yaw_offset += yaw_sum / 10;
    pitch_offset += pitch_sum / 10;
    roll_offset += roll_sum / 10;
}


//处理按键数据，对按下的按键进行记录
void App_Process_Key_Data(void)
{
    //按键任务处理逻辑
    KEY_type key = Int_KEY_Get();

    //根据按键的值进行处理，如果进行摇杆校准，将按键的值清零
   if(key == KEY_UP)
   {
        //向前飞微调，俯仰角+
        key_pitch_offset += 10;
        
   }
   else if(key == KEY_DOWN)
   {
        //向后飞微调，俯仰角-
        key_pitch_offset -= 10;
        
   }
   else if(key == KEY_LEFT)
   {
        //向左飞微调，偏航角-
        key_roll_offset -= 10;
        
   }
   else if(key == KEY_RIGHT)
   {
        //向右飞微调，偏航角+
        key_roll_offset += 10;
        
   }
   else if(key == KEY_LEFT_X)
   {
        remote_data.shutdown = 1;
   }
   else if(key == KEY_RIGHT_X)
   {
        remote_data.fix_height = 1;
   }
   else if(key == KEY_RIGHT_X_LONG)
   {
        //校准摇杆
        //处理之后thr的初始值为0，其余rew，pitch，roll的初始值为500
        App_Calibrate_Joystick();

   }
   
}
//处理摇杆数据，修正极相数据和标准值
void App_Process_Joystick_Data(void)
{
    //进入临界区
    taskENTER_CRITICAL();

    Int_Joystick_Get(&joystick);

    //1.处理范围和极性，期望范围0-1000，实际范围0-4095；未处理的极性也是反的
    joystick.thr = 1000 - (joystick.thr * 1000 / 4095);
    joystick.yaw = 1000 - (joystick.yaw * 1000 / 4095);
    joystick.pitch = 1000 - (joystick.pitch * 1000 / 4095);
    joystick.roll = 1000 - (joystick.roll * 1000 / 4095);

    //2.进行零偏校准
    joystick.thr -= thr_offset;
    joystick.yaw -= yaw_offset;
    joystick.pitch -= pitch_offset;
    joystick.roll -= roll_offset;

    //3.进行按键微调
	joystick.pitch += key_pitch_offset;
	joystick.roll += key_roll_offset;

    //4.计算的偏移值会超出范围，限制在0-1000之间
    joystick.thr = Com_Limit(joystick.thr, 0, 1000);
	joystick.yaw = Com_Limit(joystick.yaw, 0, 1000);
	joystick.pitch = Com_Limit(joystick.pitch, 0, 1000);
	joystick.roll = Com_Limit(joystick.roll, 0, 1000);
    
    //推出临界区
    taskEXIT_CRITICAL();

    //5.将处理后的数据赋值给远程数据结构体
    remote_data.thr = joystick.thr;
	remote_data.yaw = joystick.yaw;
	remote_data.pitch = joystick.pitch;
	remote_data.roll = joystick.roll;

    //debug_printf(":%d,%d,%d,%d\n", joystick.thr, joystick.yaw, joystick.pitch, joystick.roll);
}
