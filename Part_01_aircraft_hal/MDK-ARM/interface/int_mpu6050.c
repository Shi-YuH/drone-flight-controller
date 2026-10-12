#include "int_mpu6050.h"
#include "Com_debug.h"

//保存偏移量的值
int32_t gyro_offset_x = 0;
int32_t gyro_offset_y = 0;
int32_t gyro_offset_z = 0;
int32_t accel_offset_x = 0;
int32_t accel_offset_y = 0;
int32_t accel_offset_z = 0;

//写寄存器
//@brief:写寄存器
//@param:reg:寄存器地址
//@param:value:寄存器值
void Int_MPU6050_Write_Reg(uint8_t reg, uint8_t value)
{
    //写入寄存器,参数1:I2C设备句柄,参数2:从设备地址,参数3:寄存器地址,参数4:寄存器地址长度,参数5:要写入的数据,参数6:要写入的数据长度,参数7:超时时间
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_WRITE_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);

}

//读寄存器
//@brief:读寄存器
//@param:reg:寄存器地址
//@param:value:寄存器值
void Int_MPU6050_Read_Reg(uint8_t reg, uint8_t *value)
{
    //读取寄存器,参数1:I2C设备句柄,参数2:从设备地址,参数3:寄存器地址,参数4:寄存器地址长度,参数5:存储读取数据的缓冲区,参数6:要读取的数据长度,参数7:超时时间
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_READ_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, 1, 1000);

}

//零偏校准
void Int_MPU6050_Calibration_offset(void)
{
    //1.等待飞机停放平稳
    //判断飞机是否停放平稳，前后两次加速度的差值小于400 连续100次
    Accel_Data current_accel = {0};
    Accel_Data last_accel = {0};
    uint8_t count = 0;
    Int_MPU6050_Get_Acc(&current_accel);
    debug_printf("calib first cur=%d,%d,%d\r\n", current_accel.accel_x, current_accel.accel_y, current_accel.accel_z);
    while (count < 100)
    {
        Int_MPU6050_Get_Acc(&last_accel);
        if ((abs(current_accel.accel_z - last_accel.accel_z) < 400) && (abs(current_accel.accel_y - last_accel.accel_y) < 400) && (abs(current_accel.accel_x - last_accel.accel_x) < 400))
        {
            count++;
        }
        else
        {
            count = 0;
        }
        current_accel = last_accel;
        vTaskDelay(pdMS_TO_TICKS(6));
    }

    //2.进行零偏校准
    Gyro_Accel_Struct gyro_accel_data = {0};
    int32_t gyro_sum_x = 0;
    int32_t gyro_sum_y = 0;
    int32_t gyro_sum_z = 0;
    int32_t accel_sum_x = 0;
    int32_t accel_sum_y = 0;
    int32_t accel_sum_z = 0;

    for(int i = 0; i < 100; i++)
    {
        Int_MPU6050_Get_Data(&gyro_accel_data);
        gyro_sum_x += (gyro_accel_data.gyro.gyro_x - 0);
        gyro_sum_y += (gyro_accel_data.gyro.gyro_y - 0);
        gyro_sum_z += (gyro_accel_data.gyro.gyro_z - 0);
        accel_sum_x += (gyro_accel_data.accel.accel_x - 0);
        accel_sum_y += (gyro_accel_data.accel.accel_y - 0);
        accel_sum_z += (gyro_accel_data.accel.accel_z - 16384);
        vTaskDelay(pdMS_TO_TICKS(6));
    }
    gyro_offset_x = gyro_sum_x / 100;
    gyro_offset_y = gyro_sum_y / 100;
    gyro_offset_z = gyro_sum_z / 100;
    accel_offset_x = accel_sum_x / 100;
    accel_offset_y = accel_sum_y / 100;
    accel_offset_z = accel_sum_z / 100;
    
    
}

//初始化
void Int_MPU6050_Init(void)
{
    //1.重启芯片，重置所有寄存器的值 写电源管理寄存器1 DEVICE_RESET
    debug_printf("MPU init begin\r\n");
    Int_MPU6050_Write_Reg(0x6B,0x80);

    //重置完之后，0x6b的寄存器值为0x40，表示当前芯片进入低功耗
    //复位需要时间，先延时等复位完成，否则立刻读会 NACK 导致读不到 0x40
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t data = 0;
    while (data != 0x40)
    {
        Int_MPU6050_Read_Reg(0x6B, &data);
        
    }
    debug_printf("MPU reset ok\r\n");
    //唤醒芯片，写电源管理寄存器1 SLEEP_DISABLE
    Int_MPU6050_Write_Reg(0x6B,0x00);
    
    //2.选择合适的量程
    //2.1角速度量程选择为±2000dps
    Int_MPU6050_Write_Reg(0x1B,3<<3);
    //2.2加速度量程选择为±2g
    Int_MPU6050_Write_Reg(0x1C,0x00);
    
    //3.关闭中断使能，用不到中断
    Int_MPU6050_Write_Reg(0x38,0x00);

    //4.用户配置寄存器 不使用FIFO 不适用扩展的I2C
    Int_MPU6050_Write_Reg(0x6A,0x00);

    //5.设置采样频率，监控三轴加速度和三轴角速度 默认1000Hz =》1ms读取一次；在此设计为500Hz 分频系数2 =》2ms读取一次
    Int_MPU6050_Write_Reg(0x19,0x01);

    //6.设施低通滤波为184Hz 188Hz
    Int_MPU6050_Write_Reg(0x1A,0x01);

    //7.设置使用的系统时钟为添加的PLL
    Int_MPU6050_Write_Reg(0x6B,0x01);

    //8.使能加速度传感器和角速度传感器
    Int_MPU6050_Write_Reg(0x6C,0x00);

    //9.进行零偏校准
    debug_printf("MPU start calib\r\n");
    Int_MPU6050_Calibration_offset();


}

//读取角速度 需要进行零偏校准
void Int_MPU6050_Get_Gyro(Gyro_Data *gyro_data)
{
    //存储角速度寄存器的值，从0x43开始，高八位在前 XYZ顺序
    uint8_t high = 0;
    uint8_t low = 0;
    
    //读取X轴角速度
    Int_MPU6050_Read_Reg(MPU_GYRO_XOUTH_REG, &high);
    Int_MPU6050_Read_Reg(MPU_GYRO_XOUTL_REG, &low);
    gyro_data->gyro_x = ((high << 8) | low) - gyro_offset_x;
    
    //读取Y轴角速度
    Int_MPU6050_Read_Reg(MPU_GYRO_YOUTH_REG, &high);
    Int_MPU6050_Read_Reg(MPU_GYRO_YOUTL_REG, &low);
    gyro_data->gyro_y = ((high << 8) | low) - gyro_offset_y;
    
    //读取Z轴角速度
    Int_MPU6050_Read_Reg(MPU_GYRO_ZOUTH_REG, &high);
    Int_MPU6050_Read_Reg(MPU_GYRO_ZOUTL_REG, &low);
    gyro_data->gyro_z = ((high << 8) | low) - gyro_offset_z;
}

//读取加速度 抖动严重 需要零偏校准 z轴不为零
void Int_MPU6050_Get_Acc(Accel_Data *accel_data)
{
    uint8_t high = 0;
    uint8_t low = 0;

    //读取X轴加速度
    Int_MPU6050_Read_Reg(MPU_ACCEL_XOUTH_REG, &high);
    Int_MPU6050_Read_Reg(MPU_ACCEL_XOUTL_REG, &low);
    accel_data->accel_x = ((high << 8) | low) - accel_offset_x;
    
    //读取Y轴加速度
    Int_MPU6050_Read_Reg(MPU_ACCEL_YOUTH_REG, &high);
    Int_MPU6050_Read_Reg(MPU_ACCEL_YOUTL_REG, &low);
    accel_data->accel_y = ((high << 8) | low) - accel_offset_y;
    
    //读取Z轴加速度
    Int_MPU6050_Read_Reg(MPU_ACCEL_ZOUTH_REG, &high);
    Int_MPU6050_Read_Reg(MPU_ACCEL_ZOUTL_REG, &low);
    accel_data->accel_z = ((high << 8) | low) - accel_offset_z;

}

//完整数据读取,获取六轴数据
void Int_MPU6050_Get_Data(Gyro_Accel_Struct *gyro_accel_data)
{
    Int_MPU6050_Get_Acc(&gyro_accel_data->accel);
    Int_MPU6050_Get_Gyro(&gyro_accel_data->gyro);

}
