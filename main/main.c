#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "geometry_msgs/msg/twist.h"

// 包含您项目中的小车运动控制头文件
#include "car_motion.h"

// ================================================================= //
// ==================== 用户需要修改的配置 ======================== //
// ================================================================= //

// WiFi网络配置
#define WIFI_SSID "abc"
#define WIFI_PASSWORD "12345678"

// microROS Agent配置 (运行Agent的电脑IP)
#define MICRO_ROS_AGENT_IP "192.168.151.32"
#define MICRO_ROS_AGENT_PORT 8888

// ================================================================= //
// ================================================================= //

static const char *TAG = "MAIN";

// microROS 错误处理宏
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Continuing.", __LINE__, (int)temp_rc);}}

// ROS 句柄和消息
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

// /cmd_vel 话题回调函数
// 当接收到新的速度指令时，此函数被调用
void cmd_vel_subscription_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msgin;
    
    // 从消息中提取线速度(Vx)和角速度(Wz)
    // 根据 car_motion.c 中的注释，V_z 的范围是 [-5.0, 5.0]
    // Twist 消息中的 angular.z 通常代表弧度/秒，这里直接传递
    float linear_velocity_x = twist_msg->linear.x;
    float angular_velocity_z = twist_msg->angular.z;

    ESP_LOGI(TAG, "Received Vx: %.2f, Wz: %.2f", linear_velocity_x, angular_velocity_z);

    // 调用您的高层运动控制函数
    // Motion_Ctrl(V_x, V_y, V_z)
    // V_y 在您的代码中是无效的，所以我们传 0
    Motion_Ctrl(linear_velocity_x, 0, angular_velocity_z);
}

// microROS 主任务
void micro_ros_task(void *arg)
{
    ESP_LOGI(TAG, "micro-ROS task started.");

    // 配置 microROS
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // 创建一个节点，名称为 "esp32_car_node"
    RCCHECK(rclc_node_init_default(&node, "esp32_car_node", "", &support));

    // 创建一个订阅者，订阅 /cmd_vel 话题
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

    // 创建执行器
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    
    // 将订阅者添加到执行器，并关联回调函数
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_subscription_callback, ON_NEW_DATA));

    // 循环处理 microROS 事件
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000); // 10ms
    }

    // 清理资源
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rclc_support_fini(&support));

    vTaskDelete(NULL);
}

// Wi-Fi 事件处理函数
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Retrying to connect to the AP");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        
        // IP地址获取成功，配置microROS Agent并启动任务
        rmw_uros_set_custom_transport(
            true,
            (void *) MICRO_ROS_AGENT_IP,
            MICRO_ROS_AGENT_PORT
        );
        xTaskCreate(micro_ros_task, "micro_ros_task", 16000, NULL, 5, NULL);
    }
}

// Wi-Fi 初始化
void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}


void app_main(void)
{
    // 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 硬件初始化
    // 这个函数会调用 Motor_Init, Encoder_Init, 和 PwmMotor_Init
    ESP_LOGI(TAG, "Initializing car hardware...");
    Motion_Init();
    ESP_LOGI(TAG, "Car hardware initialized.");

    // 连接WiFi
    ESP_LOGI(TAG, "Connecting to WiFi...");
    wifi_init_sta();
}