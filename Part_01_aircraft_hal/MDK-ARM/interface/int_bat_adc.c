#include "int_bat_adc.h"

void Int_Bat_ADC_Init(void)
{
    //打开使能引脚
    HAL_GPIO_WritePin(BAT_ADC_EN_GPIO_Port,BAT_ADC_EN_Pin,GPIO_PIN_RESET);

    //完成ADC初始化
    HAL_ADC_Start(&hadc1);
}

float Int_Bat_ADC_Read(void)
{
    //读取ADC的14位数值
    uint32_t adc_value = HAL_ADC_GetValue(&hadc1);

    //将数值转换为电压
    float voltage = (adc_value * 3.3 / 4095 ) * 2;

    return voltage;

}
