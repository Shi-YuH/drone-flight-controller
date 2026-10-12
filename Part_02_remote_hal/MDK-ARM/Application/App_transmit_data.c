#include "App_transmit_data.h"

extern Remote_Data remote_data;
uint8_t transmit_tx_buf[TX_PLOAD_WIDTH] = {0};

uint8_t post_buff[TX_PLOAD_WIDTH] = {0};

//发送数据函数,自动切换si24r1的模式，将采集到的遥控数据打包发送到飞机
void App_Transmit_Data(void)
{
    uint32_t sum = 0;
    

    //2.发送数据,同时保证发送数据的唯一性和可靠性
    //1）帧头校验指定发送给对应的设备 唯一性
    //2）发送数据的结尾，添加校验和（将发送数据累加的值添加到数据末尾），可靠性

    //帧头校验设置为3字节，数据本身是10个字节，校验和设置为4字节
    transmit_tx_buf[0] = FRAME_HEAD_CHECK_1;
    transmit_tx_buf[1] = FRAME_HEAD_CHECK_2;
	transmit_tx_buf[2] = FRAME_HEAD_CHECK_3;

    //高八位在前
    transmit_tx_buf[3] = (remote_data.thr >> 8) & 0xFF;
	transmit_tx_buf[4] = remote_data.thr & 0xFF;

    transmit_tx_buf[5] = (remote_data.yaw >> 8) & 0xFF;
	transmit_tx_buf[6] = remote_data.yaw & 0xFF;

	transmit_tx_buf[7] = (remote_data.pitch >> 8) & 0xFF;
	transmit_tx_buf[8] = remote_data.pitch & 0xFF;

	transmit_tx_buf[9] = (remote_data.roll >> 8) & 0xFF;
	transmit_tx_buf[10] = remote_data.roll & 0xFF;

    taskENTER_CRITICAL();
	transmit_tx_buf[11] = remote_data.shutdown;
    remote_data.shutdown = 0;
	transmit_tx_buf[12] = remote_data.fix_height;
    remote_data.fix_height = 0;
    taskEXIT_CRITICAL();

    //3.校验和
    for(int i = 0; i < 13; i++)
    {
        sum += transmit_tx_buf[i];
    }
    //高位在前
    transmit_tx_buf[13] = (sum >> 24) & 0xFF;
	transmit_tx_buf[14] = (sum >> 16) & 0xFF;
	transmit_tx_buf[15] = (sum >> 8) & 0xFF;
	transmit_tx_buf[16] = sum & 0xFF;

    //debug_printf(":%d,%d,%d,%d,%d,%d\n",remote_data.thr,remote_data.yaw,remote_data.pitch,remote_data.roll,remote_data.shutdown,remote_data.fix_height);


    //4.发送数据
    //1.切换到发送模式
    Int_SI24R1_TX_Mode();
    uint8_t res = Int_SI24R1_TxPacket(transmit_tx_buf);
    //3.切换回接收模式
    Int_SI24R1_RX_Mode();
    //判断是否发送成功
    if (res == 0)
    {
        
        while (Int_SI24R1_RxPacket(post_buff) == 1 )
        {
            
        }
        //debug_printf("post_buff:%s\n",post_buff);
        
    }
    



    

}
