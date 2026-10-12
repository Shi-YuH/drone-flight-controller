#include "App_display.h"

extern Remote_Data remote_data;

extern uint8_t post_buff[TX_PLOAD_WIDTH];


void App_Display_Show_bar(uint8_t x,uint8_t y,uint8_t count)
{
    if (count < 13)
    {
        OLED_Show_CH(x,y,12 + count ,12,1);
    }
    
}

//初始化
void App_Display_Init(void)
{
    OLED_Init();
}

//显示数据
void App_Display_Show(void)
{
    //(1)X坐标 (2)Y坐标 (3)字符串 (4)字体大小 (5)模式(字白底黑)
    //将写入数据缓存
    //OLED_ShowString(0, 0, "Hello World", 12, 1);

    //1.第一行 标题
    for (uint8_t i = 0; i < 6; i++)
    {
        OLED_Show_CH(LINE1_BEGIN + 12 * i,Y0,i,12,1);
    }
    
    //2.第二行 展示2.4G通信的信道
    uint8_t buf[3] = {0};
    sprintf((char *)buf , "%03d",SI24R1_CHANNEL);
    OLED_ShowString(LINE2_BEGIN,Y1,buf,12,1);

    //第二行右边
    OLED_ShowString(LINE2_BEGIN + 64,Y1,"V:",12,1);
    OLED_ShowString(LINE2_BEGIN + 64 + 12,Y1,post_buff,12,1);

    //3.展示遥控数据
    //第三行 thr rol
    uint8_t count = 0;
    OLED_ShowString(LINE3_BEGIN,Y2,"thr:",12,1);
    if(remote_data.thr > 500)
    {
        count = (remote_data.thr - 500) / 41;
        //前一个汉字字符涂满
        App_Display_Show_bar(BAR1_BEGIN,Y2,12);
        App_Display_Show_bar(BAR2_BEGIN,Y2,count);
    }
    else
    {
        count = (remote_data.thr) / 41;
        //只图前一个字符 后一个空
        App_Display_Show_bar(BAR1_BEGIN,Y2,count);
        App_Display_Show_bar(BAR2_BEGIN,Y2,0);
    }


    OLED_ShowString(LINE3_BEGIN2,Y2,"rol:",12,1);
    if(remote_data.roll > 500)
    {
        count = (remote_data.roll - 500) / 41;
        //前一个汉字字符涂满
        App_Display_Show_bar(BAR1_BEGIN2,Y2,12);
        App_Display_Show_bar(BAR2_BEGIN2,Y2,count);
    }
    else
    {
        count = (remote_data.roll) / 41;
        //只图前一个字符 后一个空
        App_Display_Show_bar(BAR1_BEGIN2,Y2,count);
        App_Display_Show_bar(BAR2_BEGIN2,Y2,0);
    }


    //第四行 yaw pitch 
    OLED_ShowString(LINE4_BEGIN,Y3,"yaw:",12,1);
    if(remote_data.yaw > 500)
    {
        count = (remote_data.yaw - 500) / 41;
        //前一个汉字字符涂满
        App_Display_Show_bar(BAR1_BEGIN,Y3,12);
        App_Display_Show_bar(BAR2_BEGIN,Y3,count);
    }
    else
    {
        count = (remote_data.yaw) / 41;
        //只图前一个字符 后一个空
        App_Display_Show_bar(BAR1_BEGIN,Y3,count);
        App_Display_Show_bar(BAR2_BEGIN,Y3,0);
    }


    OLED_ShowString(LINE4_BEGIN2,Y3,"pit:",12,1);
    if(remote_data.pitch > 500)
    {
        count = (remote_data.pitch - 500) / 41;
        //前一个汉字字符涂满
        App_Display_Show_bar(BAR1_BEGIN2,Y3,12);
        App_Display_Show_bar(BAR2_BEGIN2,Y3,count);
    }
    else
    {
        count = (remote_data.pitch) / 41;
        //只图前一个字符 后一个空
        App_Display_Show_bar(BAR1_BEGIN2,Y3,count);
        App_Display_Show_bar(BAR2_BEGIN2,Y3,0);
    }
    //显示还要刷新缓存
    OLED_Refresh_Gram();
}
