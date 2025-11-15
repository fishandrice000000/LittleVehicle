#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdarg.h> // ***** 新增 *****: 用于处理日志函数中的可变参数

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
#include <rcl/init_options.h>

#include "geometry_msgs/msg/twist.h"
#include "rcl_interfaces/msg/log.h" // ***** 新增 *****: 包含 /rosout 话题的消息类型定义

// 包含您项目中的小车运动控制头文件
#include "car_motion.h"

// ================================================================= //
// ==================== 用户需要修改的配置 ======================== //
// ================================================================= //

// WiFi网络配置
#define WIFI_SSID "abc"
#define WIFI_PASSWORD "12345678"
// #define WIFI_SSID "lz110"
// #define WIFI_PASSWORD "lz1103300"
// #define WIFI_SSID "XYMonkeyHome"
// #define WIFI_PASSWORD "xymonkeyhome"

// microROS Agent配置 (运行Agent的电脑IP)
#define MICRO_ROS_AGENT_IP "192.168.228.32"
// #define MICRO_ROS_AGENT_IP "192.168.235.32"
// #define MICRO_ROS_AGENT_IP "192.168.31.68"
#define MICRO_ROS_AGENT_PORT "8888"

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

// ***** 新增 *****: 为日志功能添加发布者和消息变量
rcl_publisher_t log_publisher;
rcl_interfaces__msg__Log log_msg;


// ***** 新增 *****: 辅助函数，用于将日志消息发布到 /rosout 话题
void publish_log(uint8_t level, const char *name, const char *format, ...)
{
    // 检查节点是否已经初始化并有效
    if (!rcl_node_is_valid(&node)) {
        return;
    }

    va_list args;
    va_start(args, format);
    // 使用 vsnprintf 安全地将消息格式化到日志消息缓冲区中
    vsnprintf(log_msg.msg.data, log_msg.msg.capacity - 1, format, args);
    log_msg.msg.data[log_msg.msg.capacity - 1] = '\0'; // 确保字符串以空字符结尾
    log_msg.msg.size = strlen(log_msg.msg.data);
    va_end(args);

    // 设置日志消息的其他字段
    log_msg.level = level;
    strncpy(log_msg.name.data, name, log_msg.name.capacity - 1);
    log_msg.name.size = strlen(log_msg.name.data);
    
    // 发布消息
    rcl_publish(&log_publisher, &log_msg, NULL);
}

// /cmd_vel 话题回调函数
void cmd_vel_subscription_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msgin;
    
    float linear_velocity_x = twist_msg->linear.x;
    float angular_velocity_z = twist_msg->angular.z;

    // ***** 修改 *****: 用我们的网络日志函数替换 ESP_LOGI，实现日志的远程查看
    // ESP_LOGI(TAG, "Received Vx: %.2f, Wz: %.2f", linear_velocity_x, angular_velocity_z);
    publish_log(rcl_interfaces__msg__Log__INFO, rcl_node_get_name(&node), "接收到速度指令 Vx: %.2f, Wz: %.2f", linear_velocity_x, angular_velocity_z);

    // 调用您的高层运动控制函数
    Motion_Ctrl(linear_velocity_x, 0, angular_velocity_z);
}

// microROS 主任务
void micro_ros_task(void *arg)
{
    ESP_LOGI(TAG, "micro-ROS 任务已启动.");
    allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    rmw_init_options_t* rmw_init_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(MICRO_ROS_AGENT_IP, MICRO_ROS_AGENT_PORT, rmw_init_options));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // 创建一个节点
    RCCHECK(rclc_node_init_default(&node, "esp32_car_node", "", &support));

    // 创建一个 /cmd_vel 订阅者
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

    // ***** 新增 *****: 创建 /rosout 话题的发布者
    RCCHECK(rclc_publisher_init_default(
        &log_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
        "/rosout"));
        
    // ***** 新增 *****: 为日志消息中的字符串字段分配静态内存缓冲区
    static char log_buffer[256];
    log_msg.msg.data = log_buffer;
    log_msg.msg.capacity = sizeof(log_buffer);
    static char name_buffer[64];
    log_msg.name.data = name_buffer;
    log_msg.name.capacity = sizeof(name_buffer);

    // ***** 修改 *****: 更新执行器以处理2个句柄（1个订阅者，1个发布者）
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_subscription_callback, ON_NEW_DATA));

    // 通过网络日志发送一条启动消息
    publish_log(rcl_interfaces__msg__Log__INFO, rcl_node_get_name(&node), "节点已创建，执行器已启动。");

    // 无限循环处理ROS事件
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000); // 10ms
    }

    // 清理资源
    RCCHECK(rcl_publisher_fini(&log_publisher, &node)); // ***** 新增 *****
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rclc_support_fini(&support));
    vTaskDelete(NULL);
}

// Wi-Fi 事件处理函数
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "尝试重新连接到AP...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "成功获取IP地址: " IPSTR, IP2STR(&event->ip_info.ip));
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
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    wifi_config_t wifi_config = { .sta = { .ssid = WIFI_SSID, .password = WIFI_PASSWORD, }, };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "wifi_init_sta 函数执行完毕.");
}

// 主函数
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "正在初始化小车硬件...");
    Motion_Init();
    ESP_LOGI(TAG, "小车硬件初始化完成.");

    ESP_LOGI(TAG, "正在连接WiFi...");
    wifi_init_sta();
}