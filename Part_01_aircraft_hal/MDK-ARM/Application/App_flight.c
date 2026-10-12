#include "App_flight.h"

Gyro_Accel_Struct gyro_accel_data = {0};
Euler_Struct euler_data = {0};
Gyro_Data last_gyro = {0};
float yaw_sum_z = 0;

extern Remote_Data remote_data;
extern Flight_State flight_mode;

//记录当前定高的高度
extern uint16_t fix_height;

extern TaskHandle_t com_task_hander;

//电机结构体
MOTOR_STRUCT motor_top_left = {.tim = &htim3, .channel = TIM_CHANNEL_1, .speed = 0};
MOTOR_STRUCT motor_top_right = {.tim = &htim2, .channel = TIM_CHANNEL_2, .speed = 0};
MOTOR_STRUCT motor_bottom_left = {.tim = &htim4, .channel = TIM_CHANNEL_4, .speed = 0};
MOTOR_STRUCT motor_bottom_right = {.tim = &htim1, .channel = TIM_CHANNEL_3, .speed = 0};

//俯仰角PID
PID_Struct pitch_pid = {.kp = -7.00, .ki = 0.00, .kd = 0.00};
//Y轴角速度PID 对应俯仰角的内环
PID_Struct y_gyro_pid = {.kp = 3.00, .ki = 0.00, .kd = 0.50};

//横滚角PID
PID_Struct roll_pid = {.kp = -7.00, .ki = 0.00, .kd = 0.00};
//X轴角速度PID 对应横滚角的内环
PID_Struct x_gyro_pid = {.kp = 3.00, .ki = 0.00, .kd = 0.50};

//偏航角PID
PID_Struct yaw_pid = {.kp = -5.00, .ki = 0.00, .kd = 0.00};
//Z轴角速度PID 对应横滚角的内环
PID_Struct z_gyro_pid = {.kp = -5.00, .ki = 0.00, .kd = 0.0};


//定高PID结构体
PID_Struct height_pid = {.kp = -0.60, .ki = 0.00, .kd = -0.20};

//初始化 MPU6050 和 电机
void App_flight_init(void)
{
    //1.初始化MPU6050
    Int_MPU6050_Init();

    //2.启动电机
    int_motor_start(&motor_top_left);
    int_motor_start(&motor_top_right);
    int_motor_start(&motor_bottom_left);
    int_motor_start(&motor_bottom_right);

    //3.初始化VL53L0X
    int_VL53L1_init();
}

//根据测量的数据计算出欧拉角
void App_flight_get_euler_angle(void)
{
    //1.使用MPU6050的接口 获取六轴数据
    Int_MPU6050_Get_Data(&gyro_accel_data);
    
    
    //2.对角速度进行低通滤波 后续对采集到的数据使用的及时性较高 不适用傅里叶变换
    //处理过程要迅速 准确性精度要求不高 采用加权的方式计算
    gyro_accel_data.gyro.gyro_x = Common_Filter_LowPass(gyro_accel_data.gyro.gyro_x,last_gyro.gyro_x);
    gyro_accel_data.gyro.gyro_y = Common_Filter_LowPass(gyro_accel_data.gyro.gyro_y,last_gyro.gyro_y);
    gyro_accel_data.gyro.gyro_z = Common_Filter_LowPass(gyro_accel_data.gyro.gyro_z,last_gyro.gyro_z);

    last_gyro.gyro_x = gyro_accel_data.gyro.gyro_x;
    last_gyro.gyro_y = gyro_accel_data.gyro.gyro_y;
    last_gyro.gyro_z = gyro_accel_data.gyro.gyro_z;

    //打印角速度
    //debug_printf(":%d,%d,%d\n", gyro_accel_data.gyro.gyro_x, gyro_accel_data.gyro.gyro_y, gyro_accel_data.gyro.gyro_z);

    //对波动较大的加速度进行卡尔曼滤波
    gyro_accel_data.accel.accel_x = Common_Filter_KalmanFilter(&kfs[0],gyro_accel_data.accel.accel_x);
    gyro_accel_data.accel.accel_y = Common_Filter_KalmanFilter(&kfs[1],gyro_accel_data.accel.accel_y);
    gyro_accel_data.accel.accel_z = Common_Filter_KalmanFilter(&kfs[2],gyro_accel_data.accel.accel_z);

    //打印加速度
    //debug_printf(":%d,%d,%d\n", gyro_accel_data.accel.accel_x, gyro_accel_data.accel.accel_y, gyro_accel_data.accel.accel_z);

    //3.姿态解算
    ////3.1使用互补解算计算欧拉角
    //euler_data.pitch = atan2(gyro_accel_data.accel.accel_x * 1.0,gyro_accel_data.accel.accel_z) / 3.14159 * 180;
    //euler_data.roll = atan2(gyro_accel_data.accel.accel_y * 1.0,gyro_accel_data.accel.accel_z) / 3.14159 * 180;

    //偏航角只能用角速度积分
    //16为ADC的值转换为°
    //yaw_sum_z += (gyro_accel_data.gyro.gyro_z * 2000.0 / 32768.0) * 0.006;
    //euler_data.yaw = yaw_sum_z;

    ////3.2使用四元数计算欧拉角
    Common_IMU_GetEulerAngle(&gyro_accel_data,&euler_data,0.006);

    //debug_printf(":%.2f,%.2f,%.2f\n",euler_data.pitch,euler_data.roll,euler_data.yaw);
}

//根据欧拉角计算出目标的PID
void App_flight_pid_calc(void)
{
    //1.俯仰角
    //1.1传入测量值和目标值
    //外环的目标角度 如果平稳飞行，值为0 如果要进行飞行，值为遥控器的值
    //数值转换 remote_data.pitch （范围0-1000 500为中间值） 控制范围在±10°
    pitch_pid.desired = (remote_data.pitch - 500) / 50.0;
    pitch_pid.measure = euler_data.pitch;
    //内环角速度测量值 单位要一致
    y_gyro_pid.measure = (gyro_accel_data.gyro.gyro_y * 2000 / 32768.0);
    //1.2计算PID
    Common_PID_Calc_Series(&pitch_pid,&y_gyro_pid);

    //先观察内环
    //debug_printf(":%.2f,%.2f\n",y_gyro_pid.output,y_gyro_pid.err);

    //2.横滚角
    //2.1赋值目标值和测量值
    roll_pid.desired = (remote_data.roll - 500) / 50.0;
    roll_pid.measure = euler_data.roll;

    x_gyro_pid.measure = (gyro_accel_data.gyro.gyro_x * 2000 / 32768.0);

    //2.2进行PID计算
    Common_PID_Calc_Series(&roll_pid,&x_gyro_pid);
    
    //3.偏航角
    yaw_pid.desired = (remote_data.yaw - 500) / 50.0;
    yaw_pid.measure = euler_data.yaw;

    z_gyro_pid.measure = (gyro_accel_data.gyro.gyro_z * 2000 /32768.0);

    //2.2进行PID计算
    Common_PID_Calc_Series(&yaw_pid,&z_gyro_pid);

}

//执行PID控制
void App_flight_pid_control_motor(void)
{
    //1.确定当前飞机的飞行状态
    switch (flight_mode)
    {
    case FLIGHT_STATE_IDLE:
        //一旦进入空闲状态，则需要将电机速度设置为0
        motor_top_left.speed = 0;
        motor_top_right.speed = 0;
        motor_bottom_left.speed = 0;
        motor_bottom_right.speed = 0;
        break;
    case FLIGHT_STATE_NORMAL:
        //俯仰角 向前飞角速度 正误差 需要一个向后飞的反馈 -》前两个电机转的快 后两个转的慢
        motor_top_left.speed = remote_data.thr + y_gyro_pid.output - x_gyro_pid.output + z_gyro_pid.output;
        motor_top_right.speed = remote_data.thr + y_gyro_pid.output + x_gyro_pid.output - z_gyro_pid.output;
        motor_bottom_left.speed = remote_data.thr - y_gyro_pid.output - x_gyro_pid.output - z_gyro_pid.output;
        motor_bottom_right.speed = remote_data.thr - y_gyro_pid.output + x_gyro_pid.output + z_gyro_pid.output;
        break;
    case FLIGHT_STATE_FIX_HEIGHT:
        //只有在定高状态 才进行定高PID计算 并且定高时也需要平稳飞行
        motor_top_left.speed = remote_data.thr + y_gyro_pid.output - x_gyro_pid.output + z_gyro_pid.output + height_pid.output;
        motor_top_right.speed = remote_data.thr + y_gyro_pid.output + x_gyro_pid.output - z_gyro_pid.output + height_pid.output;
        motor_bottom_left.speed = remote_data.thr - y_gyro_pid.output - x_gyro_pid.output - z_gyro_pid.output + height_pid.output;
        motor_bottom_right.speed = remote_data.thr - y_gyro_pid.output + x_gyro_pid.output + z_gyro_pid.output + height_pid.output;
        
        break;
    case FLIGHT_STATE_FAIL:
        //进行故障处理 =》 一直处理 知道修改状态为IDLE
        //6ms =》速度降低3点
        motor_top_left.speed -= 3;
        motor_top_right.speed -= 3;
        motor_bottom_left.speed -= 3;
        motor_bottom_right.speed -= 3;
        if(motor_top_left.speed <= 0 && motor_top_right.speed <= 0 && motor_bottom_left.speed <= 0 && motor_bottom_right.speed <= 0)
        {
            //故障处理完成
            //发送直接任务通知
            xTaskNotifyGive(com_task_hander);
        }


        break;
    
    default:
        break;
    }

    //电机限速
    motor_top_left.speed = Com_limit(motor_top_left.speed,0,700);
    motor_top_right.speed = Com_limit(motor_top_right.speed,0,700);
    motor_bottom_left.speed = Com_limit(motor_bottom_left.speed,0,700);
    motor_bottom_right.speed = Com_limit(motor_bottom_right.speed,0,700);
    
    //安全限制 当油门设置<50 时，强制将速度设为0
    if(remote_data.thr < 50)
    {
        motor_top_left.speed = 0;
        motor_top_right.speed = 0;
        motor_bottom_left.speed = 0;
        motor_bottom_right.speed = 0;
    }

    //2.执行电机控制
    int_motor_set_speed(&motor_top_left);
    int_motor_set_speed(&motor_top_right);
    int_motor_set_speed(&motor_bottom_left);
    int_motor_set_speed(&motor_bottom_right);
}

//定高PID
void App_flight_fix_height_pid_calc(void)
{
    //1.填写目标值（按下定高功能时的高度） 和测量值（当前激光测距仪测量的高度）
    height_pid.desired = fix_height;
    height_pid.measure = int_VL53L1_get_distance();

    //2.进行单环PID计算
    Common_PID_Calc(&height_pid);
}
