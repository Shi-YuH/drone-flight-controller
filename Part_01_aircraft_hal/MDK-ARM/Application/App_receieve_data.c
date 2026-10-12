#include "App_receieve_data.h"

extern Remote_Data remote_data;

uint8_t rx_data_buffer[TX_PLOAD_WIDTH] = {0};

//表示当前链接状态
extern Remote_State remote_state;

//表示当前飞行状态
extern Flight_State flight_mode;

//表示当前定高高度
extern uint16_t fix_height;

extern uint8_t back_buf[TX_PLOAD_WIDTH];

//油门解锁状态值
Thr_State thr_state = FREE;
//MAX状态的进入时间
uint32_t max_enter_time = 0;
//MIN状态的进入时间
uint32_t min_enter_time = 0;

//重试次数
uint8_t retry_count = 0;

//接收遥控器发送的数据，解析为结构体
//返回值：0-校验成功，1-校验失败/未接收到数据
uint8_t App_receieve_data(void)
{
    memset(rx_data_buffer, 0, TX_PLOAD_WIDTH);

    uint8_t res =  Int_SI24R1_RxPacket(rx_data_buffer);
    if (res == 0)
    {
       //收到遥控数据 准备回传
        Int_SI24R1_TX_Mode();

        uint16_t cnt = 500;
        while(Int_SI24R1_TxPacket(back_buf) == 1 && cnt--)
        {

        }
        Int_SI24R1_RX_Mode();
    }
    

    if(strlen((char*)rx_data_buffer) == 0)
    {
        return 1;
    }

    //1.帧头校验
    if(rx_data_buffer[0] != FRAME_HEAD_CHECK_1 || rx_data_buffer[1] != FRAME_HEAD_CHECK_2|| rx_data_buffer[2] != FRAME_HEAD_CHECK_3)
    {
        return 1;
    }

    //2.帧尾校验
    uint32_t sum = 0;
    uint32_t rec_sum = 0;
    
    for (uint8_t i = 0; i < 13; i++)
    {
        sum += rx_data_buffer[i];
    }
    //高位在前
    rec_sum = rx_data_buffer[13] << 24 | rx_data_buffer[14] << 16 | rx_data_buffer[15] << 8 | rx_data_buffer[16];
    if(sum != rec_sum)
    {
        return 1;
    }

    //3.保存数据
    remote_data.thr = (rx_data_buffer[3] << 8) | rx_data_buffer[4];
    remote_data.yaw = (rx_data_buffer[5] << 8) | rx_data_buffer[6];
	remote_data.pitch = (rx_data_buffer[7] << 8) | rx_data_buffer[8];
	remote_data.roll = (rx_data_buffer[9] << 8) | rx_data_buffer[10];
    remote_data.shutdown = rx_data_buffer[11];
	remote_data.fix_height = rx_data_buffer[12];

    //debug_printf(":%d,%d,%d,%d,%d,%d\n", remote_data.thr, remote_data.yaw, remote_data.pitch, remote_data.roll, remote_data.shutdown, remote_data.fix_height);

    return 0;
}

//处理连接状态
//res:上一次接收数据的返回值
void App_process_connect_state(uint8_t res)
{
    if(res == 0)
    {
        // 接收数据成功一次，即接收成功，处理连接状态
        remote_state = REMOTE_STATE_CONNECTED;
        retry_count = 0;
    }
    else if(res == 1)
    {
        // 接收失败，处理断开连接
        retry_count++;
        if(retry_count >= MAX_RETRY_COUNT)
        {
            remote_state = REMOTE_STATE_DISCONNECTED;
            retry_count = 0;
        }
    }
}

//静态解锁函数,处理解锁逻辑
//返回值：0-解锁成功，1-解锁失败
static uint8_t App_process_unlock(void)
{
    //1.考虑安全问题，飞机与遥控器的油门为0
    switch (thr_state)
    {
    case FREE:
        if(remote_data.thr >= 900)
        {
            thr_state = MAX;
            max_enter_time = xTaskGetTickCount();
        }
        break;
    case MAX:
        if(remote_data.thr < 900)
        {
            //2.判断油门保持最高是否超过1s
            if(xTaskGetTickCount() - max_enter_time >= pdMS_TO_TICKS(1000))
            {
                thr_state = LEAVE_MAX;
            }
            else
            {
                thr_state = FREE;
            }
        }
        break;
    case LEAVE_MAX:
        if(remote_data.thr <= 100)
        {
            thr_state = MIN;
            min_enter_time = xTaskGetTickCount();
        }
        break;
    case MIN:
        //3.判断油门保持最低是否超过1s
        if(xTaskGetTickCount() - min_enter_time <= pdMS_TO_TICKS(1000))
        {
            if(remote_data.thr >= 100)
            {
                thr_state = FREE;
            }
        }
        else
        {
            thr_state = UNLOCK;
        }
        break;
    case UNLOCK:
        /* code */
        break;
    
    default:
        break;
    }
    if(thr_state == UNLOCK)
    {
        return 0;
    }
    return 1;
}

//处理飞行状态
void App_process_flight_state(void)
{
    //状态机逻辑实现
    //1.轮询调用判断当前所处的状态
    switch (flight_mode)
    {
    case FLIGHT_STATE_IDLE:
        // 空闲状态下的处理逻辑,解锁进入普通飞行状态
        if(App_process_unlock() == 0)
        {
            flight_mode = FLIGHT_STATE_NORMAL;
            //解锁成功后重置解锁状态
            thr_state = FREE;
            
            
        }
        break;
    case FLIGHT_STATE_NORMAL:
        // 正常飞行状态下的处理逻辑
        //判断进入定高
        if(remote_data.fix_height == 1)
        {
            flight_mode = FLIGHT_STATE_FIX_HEIGHT;
            remote_data.fix_height = 0;
            
        }
        //判断进入故障状态
        if(remote_state == REMOTE_STATE_DISCONNECTED)
        {
            flight_mode = FLIGHT_STATE_FAIL;
            
        }
        break;
    case FLIGHT_STATE_FIX_HEIGHT:
        // 固定高度飞行状态下的处理逻辑
        //判断退出定高
        if(remote_data.fix_height == 1)
        {
            flight_mode = FLIGHT_STATE_NORMAL;
            remote_data.fix_height = 0;
            //记录当前目标的高度
            fix_height = int_VL53L1_get_distance();
            
        }
        //判断进入故障状态
        if(remote_state == REMOTE_STATE_DISCONNECTED)
        {
            flight_mode = FLIGHT_STATE_FAIL;
            
        }
        break;
    case FLIGHT_STATE_FAIL:
        // 故障状态下的处理逻辑
        //处理失联故障，缓慢停止电机下降，进入空闲状态
        //等待故障处理完成 一直等待处理完成
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        flight_mode = FLIGHT_STATE_IDLE;
        break;
    
    default:
        break;
    }
}
