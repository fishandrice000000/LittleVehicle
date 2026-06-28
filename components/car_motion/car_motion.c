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
    // ***** 四轮差速驱动模型: 左侧(M1+M2) / 右侧(M3+M4) *****

    // 1. 根据线速度(V_x)和角速度(V_z)计算左右侧的速度
    float speed_left  = V_x - V_z * ROBOT_APB;
    float speed_right = V_x + V_z * ROBOT_APB;

    // 2. 同侧电机同速驱动: M1+M2(左侧), M3+M4(右侧)
    Motor_Set_Speed(speed_left, speed_left, speed_right, speed_right);
}

// 获取小车运动的速度
// Get the speed of the car's motion
void Motion_Get_Speed(car_motion_t* car)
{
    // ***** 四轮差速驱动模型反解: 左侧(M1+M2) / 右侧(M3+M4) *****

    // 1. 获取所有四个电机的速度
    float speed_m1 = 0, speed_m2 = 0, speed_m3 = 0, speed_m4 = 0;
    Motor_Get_Speed(&speed_m1, &speed_m2, &speed_m3, &speed_m4);

    // 2. 同侧电机取平均值作为该侧速度
    float left_speed  = (speed_m1 + speed_m2) / 2.0f;
    float right_speed = (speed_m3 + speed_m4) / 2.0f;

    // 3. 根据左右侧速度反解出小车的整体线速度和角速度
    //    线速度 Vx 是左右侧速度的平均值
    car->Vx = (left_speed + right_speed) / 2.0f;
    car->Vy = 0; // 差速小车没有Y方向的速度

    //    角速度 Wz 是左右侧速度差除以轮间距 (ROBOT_WIDTH_KINEMATIC)
    car->Wz = (right_speed - left_speed) / ROBOT_WIDTH_KINEMATIC;

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
