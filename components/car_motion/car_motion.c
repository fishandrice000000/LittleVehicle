#include "car_motion.h"
#include "stdio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.h"

car_motion_t micro_car;

// 控制小车运动, V_x表示控制线速度[-1.0, 1.0]，V_y无效，V_z表示控制角速度[-5.0, 5.0]。
// Control car motion, V_x control line speed [-1.0, 1.0], V_y is invalid, V_z control angular speed [-5.0, 5.0]
void Motion_Ctrl(float V_x, float V_y, float V_z)
{
    // ***** 这是为 M2(左轮)/M4(右轮) 优化的两轮差速驱动模型 *****

    // 1. 根据线速度(V_x)和角速度(V_z)计算左右轮的速度 (这部分运动学计算不变)
    float speed_left  = V_x - V_z * ROBOT_APB;
    float speed_right = V_x + V_z * ROBOT_APB;

    // 2. 根据您的新接线（左轮=M2, 右轮=M4），将速度设置到对应的电机
    //    未使用的电机(M1, M3)速度设置为0
    Motor_Set_Speed(0.0f, speed_left, 0.0f, speed_right);
}

// 获取小车运动的速度
// Get the speed of the car's motion
void Motion_Get_Speed(car_motion_t* car)
{
    // ***** 这是为 M2(左轮)/M4(右轮) 优化的两轮差速驱动模型反解 *****

    // 1. 只获取我们关心的电机M2和M4的速度
    float speed_m2 = 0, speed_m4 = 0;
    float speed_m1_dummy = 0, speed_m3_dummy = 0; // 用于接收用不到的速度值
    Motor_Get_Speed(&speed_m1_dummy, &speed_m2, &speed_m3_dummy, &speed_m4);

    // 2. 根据左右轮(M2, M4)的速度反解出小车的整体线速度和角速度
    //    线速度 Vx 是左右轮速度的平均值
    car->Vx = (speed_m2 + speed_m4) / 2.0f;
    car->Vy = 0; // 两轮差速小车没有Y方向的速度

    //    角速度 Wz 是左右轮速度差除以轮间距 (ROBOT_WIDTH_KINEMATIC)
    car->Wz = (speed_m4 - speed_m2) / ROBOT_WIDTH_KINEMATIC;
    
    if(car->Wz == 0) car->Wz = 0;
}


// 控制小车的运动状态 (此函数无需修改, 它会调用上面已优化的 Motion_Ctrl)
void Motion_Ctrl_State(uint8_t state, float speed)
{
    if (speed < 0) speed = -speed;
    if (speed > 1.0) speed = 1.0;
    switch (state)
    {
    case MOTION_STOP:
        Motion_Stop(STOP_COAST);
        break;
    case MOTION_RUN:
        Motion_Ctrl(speed, 0, 0);
        break;
    case MOTION_BACK:
        Motion_Ctrl(-speed, 0, 0);
        break;
    case MOTION_LEFT:
        Motion_Ctrl(speed, 0, speed * ROBOT_SPIN_SCALE);
        break;
    case MOTION_RIGHT:
        Motion_Ctrl(speed, 0, -speed * ROBOT_SPIN_SCALE);
        break;
    case MOTION_SPIN_LEFT:
        Motion_Ctrl(0, 0, speed * ROBOT_SPIN_SCALE);
        break;
    case MOTION_SPIN_RIGHT:
        Motion_Ctrl(0, 0, -speed * ROBOT_SPIN_SCALE);
        break;
    case MOTION_BRAKE:
        Motion_Stop(STOP_BRAKE);
        break;
    default:
        break;
    }
}


// 小车停止 (此函数无需修改)
void Motion_Stop(uint8_t brake)
{
    Motor_Stop(brake);
}


// 初始化函数 (此函数无需修改)
void Motion_Init(void)
{
    Motor_Init();
}