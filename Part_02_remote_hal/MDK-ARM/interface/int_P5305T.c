#include "int_P5305T.h"
#include "FreeRTOS.h"
#include "task.h"


//@brief:启动IP5305T电源避免自动关机
void Int_IP5305T_start(void)
{
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(100));
    HAL_GPIO_WritePin(POWER_KEY_GPIO_Port, POWER_KEY_Pin, GPIO_PIN_SET);

}
