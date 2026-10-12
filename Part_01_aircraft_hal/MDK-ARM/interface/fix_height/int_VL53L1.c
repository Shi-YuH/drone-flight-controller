#include "int_VL53L1.h"

uint16_t dev = 0x52;

// 初始化VL53L1传感器
void int_VL53L1_init(void)
{
    // 在 int_VL53L1_init() 最前面临时加：
//for (uint8_t addr = 0x02; addr < 0x7F; addr++)
//{
//    if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(addr << 1), 3, 100) == HAL_OK)
//        debug_printf("I2C2 found device at 0x%02X\r\n", addr);
//}

//vTaskDelay(pdMS_TO_TICKS(50));   // 或 100，给 AVDD 稳定 + 上电稳定


    // 1.重置传感器 XSHUT引脚拉低再拉高
    HAL_GPIO_WritePin(VX_XSHUT_GPIO_Port, VX_XSHUT_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(2));
    HAL_GPIO_WritePin(VX_XSHUT_GPIO_Port, VX_XSHUT_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(2));

//uint8_t id = 0;
//HAL_I2C_Mem_Read(&hi2c2, 0x52, 0xC0, I2C_MEMADD_SIZE_8BIT, &id, 1, 100);
//debug_printf("0xC0 = 0x%02X\r\n", id);

vTaskDelay(pdMS_TO_TICKS(50));   // 或 100，给 AVDD 稳定 + 上电稳定


    uint8_t byteData = 0;
    uint16_t wordData = 0;
    uint8_t sensorState = 0;
    /* Those basic I2C read functions can be used to check your own I2C functions */
    VL53L1_RdByte(dev, 0x010F, &byteData);
    printf("VL53L1X Model_ID: %X\n", byteData);
    VL53L1_RdByte(dev, 0x0110, &byteData);
    printf("VL53L1X Module_Type: %X\n", byteData);
    VL53L1_RdWord(dev, 0x010F, &wordData);
    printf("VL53L1X: %X\n", wordData);
    while (sensorState == 0)
    {
        debug_printf("enter\r\n");
        VL53L1X_BootState(dev, &sensorState);
        debug_printf("exit\r\n");
        vTaskDelay(2);
    }
    printf("Chip booted\n");

    /* This function must to be called to initialize the sensor with the default setting  */
    VL53L1X_SensorInit(dev);
    /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
    VL53L1X_SetDistanceMode(dev, 2);           /* 1=short, 2=long */
    VL53L1X_SetTimingBudgetInMs(dev, 20);     /* in ms possible values [20, 50, 100, 200, 500] */
    VL53L1X_SetInterMeasurementInMs(dev, 20); /* in ms, IM must be > = TB */
                                                        //  status = VL53L1X_SetOffset(dev,20); /* offset compensation in mm */
                                                        //  status = VL53L1X_SetROI(dev, 16, 16); /* minimum ROI 4,4 */
                                                        //	status = VL53L1X_CalibrateOffset(dev, 140, &offset); /* may take few second to perform the offset cal*/
                                                        //	status = VL53L1X_CalibrateXtalk(dev, 1000, &xtalk); /* may take few second to perform the xtalk cal */
    printf("VL53L1X Ultra Lite Driver Example running ...\n");
    VL53L1X_StartRanging(dev); /* This function has to be called to enable the ranging */
}

// 获取距离
// 返回值: 距离(mm)
uint16_t int_VL53L1_get_distance(void)
{
    uint16_t Distance;
    VL53L1X_GetDistance(dev, &Distance);
    return Distance;
}
