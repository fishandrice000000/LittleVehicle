#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "car_motion.h"

static const char *TAG = "MAIN";

// 运动状态枚举
typedef enum
{
    STATE_STRAIGHT_1, // 第一次直行2米
    STATE_ARC_1,      // 第一次转半圆（圆弧运动）
    STATE_STRAIGHT_2, // 第二次直行2米
    STATE_ARC_2,      // 第二次转半圆（圆弧运动）
    STATE_FINISHED    // 完成
} motion_state_t;

static motion_state_t current_state = STATE_STRAIGHT_1;
static int step_count = 0;

void app_main(void)
{
    printf("开始路径规划演示 - 两驱小车\n");
    ESP_LOGI(TAG, "路径：直行2m → 转半圆(圆弧) → 直行2m → 转半圆(圆弧)回到原地");

    // 初始化电机系统
    Motor_Init();

    // 等待系统稳定
    vTaskDelay(pdMS_TO_TICKS(2000));

    car_motion_t micro_car;

    while (1)
    {
        // 获取当前运动状态（用于调试）
        Motion_Get_Speed(&micro_car);

        switch (current_state)
        {
        case STATE_STRAIGHT_1:
            if (step_count == 0)
            {
                ESP_LOGI(TAG, "阶段1: 直行2米");
                // 设置直行速度
                Motion_Ctrl(0.4, 0, 0); // 线速度0.4m/s，角速度0
            }
            step_count++;
            ESP_LOGI(TAG, "直行中: %.1f秒, Vx=%.2f", step_count * 0.1, micro_car.Vx);

            // 直行5秒 ≈ 2米 (0.4m/s × 5s = 2m)
            if (step_count >= 50)
            { // 50 × 100ms = 5秒
                ESP_LOGI(TAG, "直行2米完成");
                Motion_Stop(STOP_COAST);
                vTaskDelay(pdMS_TO_TICKS(1000));
                current_state = STATE_ARC_1;
                step_count = 0;
            }
            break;

        case STATE_ARC_1:
            if (step_count == 0)
            {
                ESP_LOGI(TAG, "阶段2: 转半圆(圆弧运动)");
                // 同时给线速度和角速度，做圆弧运动
                // 线速度0.3m/s，角速度0.6rad/s
                Motion_Ctrl(0.3, 0, 0.6);
            }
            step_count++;
            ESP_LOGI(TAG, "圆弧运动中: %.1f秒, Vx=%.2f, Wz=%.2f",
                     step_count * 0.1, micro_car.Vx, micro_car.Wz);

            // 圆弧运动时间需要计算：180度 = π弧度，角速度0.6rad/s
            // 时间 = π / 0.6 ≈ 5.23秒
            if (step_count >= 52)
            { // 52 × 100ms = 5.2秒
                ESP_LOGI(TAG, "转半圆完成");
                Motion_Stop(STOP_COAST);
                vTaskDelay(pdMS_TO_TICKS(1000));
                current_state = STATE_STRAIGHT_2;
                step_count = 0;
            }
            break;

        case STATE_STRAIGHT_2:
            if (step_count == 0)
            {
                ESP_LOGI(TAG, "阶段3: 直行2米");
                Motion_Ctrl(0.4, 0, 0);
            }
            step_count++;
            ESP_LOGI(TAG, "直行中: %.1f秒, Vx=%.2f", step_count * 0.1, micro_car.Vx);

            if (step_count >= 50)
            { // 5秒
                ESP_LOGI(TAG, "直行2米完成");
                Motion_Stop(STOP_COAST);
                vTaskDelay(pdMS_TO_TICKS(1000));
                current_state = STATE_ARC_2;
                step_count = 0;
            }
            break;

        case STATE_ARC_2:
            if (step_count == 0)
            {
                ESP_LOGI(TAG, "阶段4: 转半圆(圆弧运动)回到起点");
                // 与第一次圆弧运动方向相反
                Motion_Ctrl(0.3, 0, -0.6); // 角速度取负值
            }
            step_count++;
            ESP_LOGI(TAG, "圆弧运动中: %.1f秒, Vx=%.2f, Wz=%.2f",
                     step_count * 0.1, micro_car.Vx, micro_car.Wz);

            if (step_count >= 52)
            { // 5.2秒
                ESP_LOGI(TAG, "转半圆完成，回到起点！");
                Motion_Stop(STOP_COAST);
                current_state = STATE_FINISHED;

                // 任务完成
                ESP_LOGI(TAG, "=== 路径规划任务完成！小车已回到起点 ===");
            }
            break;

        case STATE_FINISHED:
            // 保持停止状态
            if (step_count % 20 == 0)
            { // 每2秒输出一次
                ESP_LOGI(TAG, "任务已完成，小车停在起点");
            }
            step_count++;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms控制周期
    }
}