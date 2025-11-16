#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdarg.h> 

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
#include "rcl_interfaces/msg/log.h" 

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

// micro-ROS连接重试配置
#define MAX_RETRY_ATTEMPTS   5
#define RETRY_DELAY_MS     2000

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

// 为日志功能添加发布者和消息变量
rcl_publisher_t log_publisher;
rcl_interfaces__msg__Log log_msg;


// 辅助函数，用于将日志消息发布到 /rosout 话题
void publish_log(uint8_t level, const char *file, const char *function, int line, const char *name, const char *format, ...)
{
    if (!rcl_node_is_valid(&node)) {
        return;
    }

    // --- 时间戳填充 ---
    int64_t stamp_nanos = 0;
    // 检查时间是否已经和Agent同步
    if (rmw_uros_epoch_synchronized()) {
        // 获取同步后的纳秒时间戳
        stamp_nanos = rmw_uros_epoch_nanos();
    }
    log_msg.stamp.sec = stamp_nanos / 1000000000;
    log_msg.stamp.nanosec = stamp_nanos % 1000000000;
    
    // --- 其他字段填充 ---
    log_msg.level = level;
    strncpy(log_msg.name.data, name, log_msg.name.capacity - 1);
    log_msg.name.size = strlen(log_msg.name.data);
    strncpy(log_msg.file.data, file, log_msg.file.capacity - 1);
    log_msg.file.size = strlen(log_msg.file.data);
    strncpy(log_msg.function.data, function, log_msg.function.capacity - 1);
    log_msg.function.size = strlen(log_msg.function.data);
    log_msg.line = line;

    // --- 消息内容格式化 ---
    va_list args;
    va_start(args, format);
    vsnprintf(log_msg.msg.data, log_msg.msg.capacity - 1, format, args);
    log_msg.msg.size = strlen(log_msg.msg.data);
    va_end(args);
    
    RCSOFTCHECK(rcl_publish(&log_publisher, &log_msg, NULL));
}

#define PUBLISH_LOG_INFO(NODE_NAME, FORMAT, ...) \
    publish_log(rcl_interfaces__msg__Log__INFO, __FILE__, __func__, __LINE__, NODE_NAME, FORMAT, ##__VA_ARGS__)

// /cmd_vel 话题回调函数
void cmd_vel_subscription_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msgin;
    
    float linear_velocity_x = twist_msg->linear.x;
    float angular_velocity_z = twist_msg->angular.z;

    PUBLISH_LOG_INFO(rcl_node_get_name(&node), "接收到速度指令 Vx: %.2f, Wz: %.2f", linear_velocity_x, angular_velocity_z);

    // 调用您的高层运动控制函数
    Motion_Ctrl(linear_velocity_x, 0, angular_velocity_z);
}

// microROS 主任务
void micro_ros_task(void *arg)
{
    ESP_LOGI(TAG, "micro-ROS 任务已启动.");
    
    bool is_initialized = false;
    // 增加了重试循环
    for (int attempts = 1; attempts <= MAX_RETRY_ATTEMPTS; attempts++) {
        ESP_LOGI(TAG, "micro-ROS 初始化尝试 #%d/%d", attempts, MAX_RETRY_ATTEMPTS);
        
        // 使用 do-while(0) 结构方便地处理初始化序列中的错误
        do {
            allocator = rcl_get_default_allocator();
            rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
            if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) break;

            rmw_init_options_t* rmw_init_options = rcl_init_options_get_rmw_init_options(&init_options);
            if (rmw_uros_options_set_udp_address(MICRO_ROS_AGENT_IP, MICRO_ROS_AGENT_PORT, rmw_init_options) != RMW_RET_OK) break;

            if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) break;

            // 在初始化支持后，显式等待与Agent的时间同步
            if (rmw_uros_sync_session(1000) != RMW_RET_OK) {
                ESP_LOGE(TAG, "与Agent的时间同步失败。");
                break; // 跳出do-while，触发重试逻辑
            }

            if (rclc_node_init_default(&node, "esp32_car_node", "", &support) != RCL_RET_OK) break;

            if (rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel") != RCL_RET_OK) break;

            rmw_qos_profile_t qos_profile = rmw_qos_profile_services_default;
            qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
            if (rclc_publisher_init(&log_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log), "/rosout", &qos_profile) != RCL_RET_OK) break;
            
            static char log_buffer[256];
            log_msg.msg.data = log_buffer;
            log_msg.msg.capacity = sizeof(log_buffer);
            static char name_buffer[64];
            log_msg.name.data = name_buffer;
            log_msg.name.capacity = sizeof(name_buffer);
            static char file_buffer[128];
            log_msg.file.data = file_buffer;
            log_msg.file.capacity = sizeof(file_buffer);
            static char function_buffer[128];
            log_msg.function.data = function_buffer;
            log_msg.function.capacity = sizeof(function_buffer);

            if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) break;
            
            if (rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_subscription_callback, ON_NEW_DATA) != RCL_RET_OK) break;

            is_initialized = true; // 所有步骤成功

        } while (0);

        if (is_initialized) {
            ESP_LOGI(TAG, "micro-ROS 初始化成功。");
            break; // 跳出重试循环
        }

        // 如果初始化失败，清理资源并等待重试
        ESP_LOGE(TAG, "micro-ROS 初始化失败，正在清理...");
        RCSOFTCHECK(rclc_executor_fini(&executor));
        RCSOFTCHECK(rcl_publisher_fini(&log_publisher, &node));
        RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
        RCSOFTCHECK(rcl_node_fini(&node));
        RCSOFTCHECK(rclc_support_fini(&support));
        
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
    }

    // 如果所有重试都失败，则终止任务
    if (!is_initialized) {
        ESP_LOGE(TAG, "无法初始化 micro-ROS。任务中止。");
        vTaskDelete(NULL);
    }
    
    PUBLISH_LOG_INFO(rcl_node_get_name(&node), "节点已创建，执行器已启动。");

    // 无限循环处理ROS事件
    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000); // 10ms
    }

    // 清理资源 (正常情况下不会执行到这里)
    RCCHECK(rcl_publisher_fini(&log_publisher, &node)); 
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