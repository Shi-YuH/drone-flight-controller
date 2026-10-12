#include "App_freeRTOS_Task.h"

//电源管理任务
void Power_Task(void *pvParameters);

TaskHandle_t power_task_hander;
#define POWER_TASK_STACK_SIZE 128
#define POWER_TASK_PRIORITY 4
#define POWER_TASK_PERIOD_MS 10000

//通信任务
void Com_Task(void *pvParameters);
TaskHandle_t com_task_hander;
#define COM_TASK_STACK_SIZE 128
#define COM_TASK_PRIORITY 4
#define COM_TASK_PERIOD_MS 10

//按键任务
void Key_Task(void *pvParameters);
TaskHandle_t key_task_hander;
#define KEY_TASK_STACK_SIZE 128
#define KEY_TASK_PRIORITY 2
#define KEY_TASK_PERIOD_MS 20

//摇杆任务
void Joystick_Task(void *pvParameters);
TaskHandle_t joystick_task_hander;
#define JOYSTICK_TASK_STACK_SIZE 128
#define JOYSTICK_TASK_PRIORITY 2
#define JOYSTICK_TASK_PERIOD_MS 20

//显示屏任务
void Display_Task(void *pvParameters);
TaskHandle_t display_task_hander;
#define DISPLAY_TASK_STACK_SIZE 128
#define DISPLAY_TASK_PRIORITY 1
#define DISPLAY_TASK_PERIOD_MS 100

// @brief:启动FreeRTOS任务
void App_freeRTOS_start(void)
{
    //1.创建电源管理任务
    xTaskCreate(Power_Task, "Power_Task", POWER_TASK_STACK_SIZE, NULL, POWER_TASK_PRIORITY, &power_task_hander);

    //2.创建通信任务
    xTaskCreate(Com_Task, "Com_Task", COM_TASK_STACK_SIZE, NULL, COM_TASK_PRIORITY, &com_task_hander);
    
    //3.创建按键任务
    xTaskCreate(Key_Task, "Key_Task", KEY_TASK_STACK_SIZE, NULL, KEY_TASK_PRIORITY, &key_task_hander);

    //4.创建摇杆任务
    xTaskCreate(Joystick_Task, "Joystick_Task", JOYSTICK_TASK_STACK_SIZE, NULL, JOYSTICK_TASK_PRIORITY, &joystick_task_hander);

    //5.创建显示屏任务
    xTaskCreate(Display_Task, "Display_Task", DISPLAY_TASK_STACK_SIZE, NULL, DISPLAY_TASK_PRIORITY, &display_task_hander);
    //3.启动调度器
    vTaskStartScheduler();

    //调度器一旦启动成功就不会执行到这里；执行到这里说明启动失败（通常是堆不够）
    debug_printf("Scheduler start failed\r\n");
}

void Power_Task(void *pvParameters)
{
    //获取当前基准时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        //每10s启动一次电源，避免关机
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(POWER_TASK_PERIOD_MS));

         Int_IP5305T_start();
    }
}

void Key_Task(void *pvParameters)
{
    //获取当前基准时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        App_Process_Key_Data();
        //每20ms执行一次按键任务
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(KEY_TASK_PERIOD_MS));
    }
}

//摇杆任务
void Joystick_Task(void *pvParameters)
{
    //获取当前基准时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    //初始化摇杆ADC
    Int_Joystick_Init();

    while(1)
    {
        //摇杆任务处理逻辑

        App_Process_Joystick_Data();

        
        //每50ms执行一次摇杆任务
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(JOYSTICK_TASK_PERIOD_MS));
    }
}

void Com_Task(void *pvParameters)
{
    //获取当前基准时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        //将打包完成的诗句发送到飞机
        App_Transmit_Data();
        //每10ms执行一次通信任务
        //6ms执行一次 =>vTaskDelayUntil()算上函数运行的时间 总计延迟6ms
        //比如此次发送是在1ms的位置:重试发送延迟2ms => 下一次发送还是1ms(7ms)接收方还是没有启动
        // 在第一次匹配上完成收发之后 两端一起延迟一个时间 再次收发
        vTaskDelay(pdMS_TO_TICKS(COM_TASK_PERIOD_MS));
        
    }
}

void Display_Task(void *pvParameters)
{
    //获取当前基准时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    //初始化显示屏
    App_Display_Init();
    while(1)
    {
        //显示屏任务处理逻辑
        App_Display_Show();
        
        //每100ms执行一次显示屏任务
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DISPLAY_TASK_PERIOD_MS));
        
    }
}
