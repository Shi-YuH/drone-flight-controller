#include "App_freeRTOS_Task.h"


//内存管理，始终使用一个结构体，循环使用

//LED结构体
LED_STRUCT led_top_left = {.port = LED1_GPIO_Port, .pin = LED1_Pin};
LED_STRUCT led_top_right = {.port = LED2_GPIO_Port, .pin = LED2_Pin};
LED_STRUCT led_bottom_right = {.port = LED3_GPIO_Port, .pin = LED3_Pin};
LED_STRUCT led_bottom_left = {.port = LED4_GPIO_Port, .pin = LED4_Pin};

//表示当前链接状态
Remote_State remote_state = REMOTE_STATE_DISCONNECTED;

//表示当前飞行模式
Flight_State flight_mode = FLIGHT_STATE_IDLE;

//扩展获取遥控命令
Remote_Data remote_data = {.yaw = 500, .pitch = 500, .roll = 500, .thr = 0, .fix_height = 0,.shutdown = 0};

//按下定高时的定高高度
uint16_t fix_height = 0;

//电池电压 =>数组数据
uint8_t back_buf[TX_PLOAD_WIDTH] = {0};

//电源管理任务
void Power_Task(void *pvParameters);
TaskHandle_t power_task_hander;
#define POWER_TASK_STACK_SIZE 128
#define POWER_TASK_PRIORITY 4
#define POWER_TASK_PERIOD 10000

//飞行控制任务
void Flight_Control_Task(void *pvParameters);
TaskHandle_t flight_control_task_hander;
#define FLIGHT_CONTROL_TASK_STACK_SIZE 128
#define FLIGHT_CONTROL_TASK_PRIORITY 3
#define FLIGHT_CONTROL_TASK_PERIOD 10

//LED灯控任务
void LED_Task(void *pvParameters);
TaskHandle_t led_task_hander;
#define LED_TASK_STACK_SIZE 128
#define LED_TASK_PRIORITY 1
#define LED_TASK_PERIOD 100

//通信任务
void Com_Task(void *pvParameters);
TaskHandle_t com_task_hander;
#define COM_TASK_STACK_SIZE 128
#define COM_TASK_PRIORITY 4
#define COM_TASK_PERIOD 10

// @brief:启动FreeRTOS任务
void App_freeRTOS_start(void)
{
    //1.创建电源管理任务
    xTaskCreate(Power_Task, "Power_Task", POWER_TASK_STACK_SIZE, NULL, POWER_TASK_PRIORITY, &power_task_hander);

    //2.创建飞行控制任务
    xTaskCreate(Flight_Control_Task, "Flight_Control_Task", FLIGHT_CONTROL_TASK_STACK_SIZE, NULL, FLIGHT_CONTROL_TASK_PRIORITY, &flight_control_task_hander);

    //3.创建LED灯控任务
    xTaskCreate(LED_Task, "LED_Task", LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, &led_task_hander);

    //4.创建通信任务
    xTaskCreate(Com_Task, "Com_Task", COM_TASK_STACK_SIZE, NULL, COM_TASK_PRIORITY, &com_task_hander);

    //2.启动调度器
    vTaskStartScheduler();

    //调度器一旦启动成功就不会执行到这里；执行到这里说明启动失败（通常是堆不够）
    debug_printf("Scheduler start failed\r\n");
}


// @brief:电源管理任务
void Power_Task(void *pvParameters)
{
    //获取当前基准时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        //每10s启动一次电源，避免关机
        //vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(POWER_TASK_PERIOD));

        // Int_IP5305T_start();

        //使用直接任务通知，执行10s重启一次
        //一直等待任务通知，直到收到通知res = 1 或者 超时res = 0
        uint32_t res = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(POWER_TASK_PERIOD));
        //收到关机通知
        if(res != 0)
        {
            Int_IP5305T_shutdown();
        }
        else
        {
            Int_IP5305T_start();
        }
    }
}


//飞行控制任务
void Flight_Control_Task(void *pvParameters)
{
    //获取当前基准时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint16_t count = 0;
    debug_printf("Flight start\r\n");
    App_flight_init();
    debug_printf("MPU6050 init done\r\n");
    while(1)
    {
        // TODO: 实现飞行控制逻辑
        //1.根据测量值计算欧拉角
        App_flight_get_euler_angle();

        //2.根据欧拉角计算PID
        App_flight_pid_calc();

        //3.判断定高
        if(flight_mode == FLIGHT_STATE_FIX_HEIGHT)
        {
            count++;
            if(count >= 4 )
            {
                App_flight_pid_control_motor();
                count = 0;
            }
        }


        //3.根据PID对电机进行控制
        App_flight_pid_control_motor();

        ////4.打印激光测距仪得到的测量值
        //uint16_t distance = int_VL53L1_get_distance();

        //debug_printf("Distance: %d\r\n", distance);

        //每10ms执行一次飞行控制
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(FLIGHT_CONTROL_TASK_PERIOD));
    }
}


void LED_Task(void *pvParameters)
{
    //获取当前基准时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t count = 0;
    while(1)
    {
        count++;
        // TODO: 实现LED控制逻辑

        //前两个灯表示连接状态
        //1.判断当前的链接状态
        if(remote_state == REMOTE_STATE_CONNECTED)
        {
            //如果当前是连接状态，则打开前两个LED灯
            int_led_on(&led_top_left);
            int_led_on(&led_top_right);
        }
        else if (remote_state == REMOTE_STATE_DISCONNECTED)
        {
            //如果当前是断开状态，则关闭前两个LED灯
            int_led_off(&led_top_left);
            int_led_off(&led_top_right);
        }

        //后两个灯表示飞行状态
        //1.判断当前的飞行状态
        if(flight_mode == FLIGHT_STATE_IDLE)
        {
            //如果当前是空闲状态，则后两个LED灯慢闪烁，500ms闪烁一次
            if(count % 5 == 0)
            {
                int_led_toggle(&led_bottom_left);
                int_led_toggle(&led_bottom_right);
            }
        }
        else if (flight_mode == FLIGHT_STATE_NORMAL)
        {
            //如果当前是飞行状态，则后两个LED灯快闪烁，200ms闪烁一次
            if(count % 1 == 0)
            {
                int_led_toggle(&led_bottom_left);
                int_led_toggle(&led_bottom_right);
            }
        }
        else if (flight_mode ==FLIGHT_STATE_FIX_HEIGHT)
        {
            //如果当前是定高状态，则常亮后两个LED灯
            int_led_on(&led_bottom_left);
            int_led_on(&led_bottom_right);
        }
        else if (flight_mode == FLIGHT_STATE_FAIL)
        {
            //如果当前是故障状态，则关闭后两个LED灯
            int_led_off(&led_bottom_left);
            int_led_off(&led_bottom_right);
        }

        //重置计数器
        if(count == 10)
        {
            count = 0;
        }


        //每100ms执行一次LED任务
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LED_TASK_PERIOD));
    }
}


//通信任务
void Com_Task(void *pvParameters)
{
    //获取当前基准时间
    TickType_t xLastWakeTime = xTaskGetTickCount();
    //初始化电池电压测量
    Int_Bat_ADC_Init();
    while(1)
    {
        // TODO: 实现通信任务逻辑
        //1.接收数据
        uint8_t res = App_receieve_data();

        //2.根据接收数据的返回值处理连接状态
        App_process_connect_state(res);

        //3.处理关机命令
        if(remote_data.shutdown == 1)
        {
            //Int_IP5305T_shutdown();
            //使用freeRTOS直接任务通知，通知电源任务，使其执行关机操作
            xTaskNotifyGive(power_task_hander);
        }

        //4.处理飞机的飞行状态 =》如果进入故障状态 会一直等待任务通知
        App_process_flight_state();

        //5.准备回传电池电压
        float voltage = Int_Bat_ADC_Read();
        //debug_printf(":%.2f\n",voltage);
        sprintf((char*)back_buf,"%.2f",voltage);


        //每100ms执行一次通信任务
        vTaskDelay(pdMS_TO_TICKS(COM_TASK_PERIOD));
    }
}

