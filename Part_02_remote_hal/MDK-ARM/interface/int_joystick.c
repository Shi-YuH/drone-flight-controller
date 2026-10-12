#include "int_joystick.h"


uint16_t adc_buffer[4] = {0};
//初始化ADC摇杆，打开ADC
void Int_Joystick_Init(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 4);
}

//获取摇杆数据,joystick:摇杆数据结构体指针
//保存摇杆数据到joystick结构体中
void Int_Joystick_Get(Joystick_Struct *joystick)
{
    //DMA不依赖CPU计算，ADC获取到的数据实时保存在ADC_BUFFER中
    //顺序是自定义的RANK1234，要对齐
    joystick->thr = adc_buffer[0];
    joystick->yaw = adc_buffer[1];
    joystick->pitch = adc_buffer[2];
    joystick->roll = adc_buffer[3];

}
