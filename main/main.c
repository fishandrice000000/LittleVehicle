#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdarg.h> 
#include <math.h>   // 用于里程计姿态计算（cosf/sinf）

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"   // 用于获取微秒级时间戳（里程计积分用）

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/init_options.h>

#include "geometry_msgs/msg/twist.h"
#include "rcl_interfaces/msg/log.h" 
#include "nav_msgs/msg/odometry.h"  // 里程计：Odometry 消息类型

// 小车运动控制头文件（包含运动学与电机控制接口）
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

// ===== micro-ROS 错误处理宏（统一检查返回值并输出日志） =====
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Continuing.", __LINE__, (int)temp_rc);}}

// ===== ROS 句柄与消息对象（全局） =====
rclc_executor_t executor;          // rclc 执行器：驱动回调执行
rclc_support_t support;            // rclc 支持结构体：封装 init/context
rcl_allocator_t allocator;         // 内存分配器
rcl_node_t node;                   // micro-ROS 节点
rcl_subscription_t subscriber;     // /cmd_vel 订阅者
geometry_msgs__msg__Twist msg;     // 订阅用 Twist 消息缓存

// ===== 日志发布者（/rosout） =====
rcl_publisher_t log_publisher;     
rcl_interfaces__msg__Log log_msg;  // rcl_interfaces/Log 消息，用于发布到 /rosout

// ===== 里程计发布者（/odom）与内部状态 =====
rcl_publisher_t odom_publisher;    // /odom 发布者
nav_msgs__msg__Odometry odom_msg;  // Odometry 消息缓存

// 里程计内部积分状态（以“odom”坐标系为参考）
static float odom_x = 0.0f;        // 小车在平面上的 x 位置（m）
static float odom_y = 0.0f;        // 小车在平面上的 y 位置（m）
static float odom_theta = 0.0f;    // 小车朝向 yaw 角（rad）

// ===== cmd_vel 看门狗辅助变量 =====
static int64_t last_cmd_time_us = 0;         // 最近一次收到 /cmd_vel 的时间（微秒，用于看门狗超时停车）

// ============================================================================
// 日志辅助函数：发布 rcl_interfaces::msg::Log 到 /rosout
// ============================================================================
// 说明：
//   - level     : 日志等级（INFO/WARN/ERROR 等枚举值）
//   - file      : 源文件名（__FILE__）
//   - function  : 函数名（__func__）
//   - line      : 行号（__LINE__）
//   - name      : 节点名称（rcl_node_get_name）
//   - format... : printf 风格的格式化字符串
// ============================================================================
void publish_log(uint8_t level, const char *file, const char *function, int line, const char *name, const char *format, ...)
{
    if (!rcl_node_is_valid(&node)) {
        return;
    }

    // --- 时间戳填充（使用与 Agent 同步的时间） ---
    int64_t stamp_nanos = 0;
    if (rmw_uros_epoch_synchronized()) {
        // 获取同步后的纳秒时间戳
        stamp_nanos = rmw_uros_epoch_nanos();
    }
    log_msg.stamp.sec = stamp_nanos / 1000000000;
    log_msg.stamp.nanosec = stamp_nanos % 1000000000;
    
    // --- 基本字段填充 ---
    log_msg.level = level;
    strncpy(log_msg.name.data, name, log_msg.name.capacity - 1);
    log_msg.name.size = strlen(log_msg.name.data);
    strncpy(log_msg.file.data, file, log_msg.file.capacity - 1);
    log_msg.file.size = strlen(log_msg.file.data);
    strncpy(log_msg.function.data, function, log_msg.function.capacity - 1);
    log_msg.function.size = strlen(log_msg.function.data);
    log_msg.line = line;

    // --- 文本消息内容格式化 ---
    va_list args;
    va_start(args, format);
    vsnprintf(log_msg.msg.data, log_msg.msg.capacity - 1, format, args);
    log_msg.msg.size = strlen(log_msg.msg.data);
    va_end(args);
    
    RCSOFTCHECK(rcl_publish(&log_publisher, &log_msg, NULL));
}

// 日志宏封装，简化调用：默认使用 INFO 等级
#define PUBLISH_LOG_INFO(NODE_NAME, FORMAT, ...) \
    publish_log(rcl_interfaces__msg__Log__INFO, __FILE__, __func__, __LINE__, NODE_NAME, FORMAT, ##__VA_ARGS__)

// ============================================================================
// /cmd_vel 订阅回调：接收速度指令并调用底层运动控制
// ============================================================================
// 说明：
//   - 订阅 geometry_msgs/Twist 类型的 /cmd_vel
//   - 使用 linear.x 作为车体前进线速度 Vx
//   - 使用 angular.z 作为车体绕 Z 轴角速度 Wz
//   - 调用 Motion_Ctrl(Vx, 0, Wz) 完成差速控制
// ============================================================================
void cmd_vel_subscription_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msgin;
    
    float linear_velocity_x = twist_msg->linear.x;
    float angular_velocity_z = twist_msg->angular.z;

    PUBLISH_LOG_INFO(rcl_node_get_name(&node), "接收到速度指令 Vx: %.2f, Wz: %.2f", linear_velocity_x, angular_velocity_z);

    // 记录最近一次收到 /cmd_vel 的时间（用于看门狗超时停车）
    last_cmd_time_us = esp_timer_get_time();

    // 调用小车高层运动控制接口：差速模型封装在 Motion_Ctrl 内部
    Motion_Ctrl(linear_velocity_x, 0, angular_velocity_z);
}

// ============================================================================
// micro-ROS 主任务：负责初始化、连接 Agent、创建节点与通讯对象，并运行主循环
// ============================================================================
// 流程概述：
//   1. 配置并初始化 rcl/micro-ROS 支持（包含 UDP Agent 地址）
//   2. 同步时间（rmw_uros_sync_session）
//   3. 创建节点、订阅者（/cmd_vel）、日志发布者（/rosout）、里程计发布者（/odom）
//   4. 初始化 rclc 执行器并注册订阅回调
//   5. 在 while(1) 中：
//      - 轮询 executor 处理 ROS 事件
//      - 从编码器读取速度、积分位姿，周期性发布 /odom
//      - 检查 /cmd_vel 看门狗超时，必要时主动停止小车
// ============================================================================
void micro_ros_task(void *arg)
{
    ESP_LOGI(TAG, "micro-ROS 任务已启动.");
    
    bool is_initialized = false;
    // 增加了重试循环（网络/Agent 不稳定时自动重连）
    for (int attempts = 1; attempts <= MAX_RETRY_ATTEMPTS; attempts++) {
        ESP_LOGI(TAG, "micro-ROS 初始化尝试 #%d/%d", attempts, MAX_RETRY_ATTEMPTS);
        
        // 使用 do-while(0) 方便在任一步骤失败时统一跳出
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
                break; // 跳出 do-while，触发重试逻辑
            }

            if (rclc_node_init_default(&node, "esp32_car_node", "", &support) != RCL_RET_OK) break;

            if (rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel") != RCL_RET_OK) break;

            // /rosout 日志发布者，使用 TRANSIENT_LOCAL 以便后来的订阅者也能接收旧日志
            rmw_qos_profile_t qos_profile = rmw_qos_profile_services_default;
            qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
            if (rclc_publisher_init(&log_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log), "/rosout", &qos_profile) != RCL_RET_OK) break;
            
            // 为 Log 字符串字段分配静态缓存
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

            // 初始化 /odom 发布者和 frame_id / child_frame_id
            if (rclc_publisher_init_default(
                    &odom_publisher,
                    &node,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                    "/odom") != RCL_RET_OK) break;

            static char odom_frame_id[16];
            static char odom_child_frame_id[16];

            odom_msg.header.frame_id.data = odom_frame_id;
            odom_msg.header.frame_id.capacity = sizeof(odom_frame_id);
            strncpy(odom_msg.header.frame_id.data, "odom", odom_msg.header.frame_id.capacity - 1);
            odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);

            odom_msg.child_frame_id.data = odom_child_frame_id;
            odom_msg.child_frame_id.capacity = sizeof(odom_child_frame_id);
            strncpy(odom_msg.child_frame_id.data, "base_link", odom_msg.child_frame_id.capacity - 1);
            odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);

            // 初始化 rclc 执行器，并注册 /cmd_vel 订阅回调
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
        RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
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

    // 里程计时间基准（微秒）
    int64_t last_odom_time_us = esp_timer_get_time();

    // =====================================================================
    // 主循环：
    //   - 处理 ROS 通讯事件
    //   - 周期性读取车体速度，积分位姿，发布 /odom
    //   - 检查 /cmd_vel 看门狗超时，必要时主动停止小车
    // =====================================================================
    while (1)
    {
        // 非阻塞方式轮询回调，超时时间 100ms
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        // -------------------- 里程计计算与发布 --------------------
        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_odom_time_us) / 1000000.0f;  // 时间间隔（秒）
        if (dt < 0.0f) {
            dt = 0.0f;
        }
        if (dt > 1.0f) {
            // 极端卡顿场景下防止一次积分过大（正常循环不会到 1s）
            dt = 1.0f;
        }
        last_odom_time_us = now_us;

        // 1. 通过已有接口获取车体速度（Vx, Wz）
        car_motion_t car;
        Motion_Get_Speed(&car);
        float vx = car.Vx;    // 车体坐标系前进方向速度 m/s
        float wz = car.Wz;    // 绕 Z 轴角速度 rad/s

        // 2. 简单双轮差速里程计积分（在 odom 平面坐标系中积分）
        odom_theta += wz * dt;

        float cos_t = cosf(odom_theta);
        float sin_t = sinf(odom_theta);
        odom_x += vx * cos_t * dt;
        odom_y += vx * sin_t * dt;

        // 3. 填充 Odometry 消息并发布
        int64_t stamp_nanos = 0;
        if (rmw_uros_epoch_synchronized()) {
            stamp_nanos = rmw_uros_epoch_nanos();
        } else {
            // 如果未同步与 Agent 的时间，则退化为本地时间估计
            stamp_nanos = now_us * 1000;
        }
        odom_msg.header.stamp.sec = stamp_nanos / 1000000000;
        odom_msg.header.stamp.nanosec = stamp_nanos % 1000000000;

        // 位置（二维平面，Z 维固定为 0）
        odom_msg.pose.pose.position.x = odom_x;
        odom_msg.pose.pose.position.y = odom_y;
        odom_msg.pose.pose.position.z = 0.0f;

        // 姿态（yaw -> Z 轴旋转四元数）
        float half_theta = odom_theta * 0.5f;
        float cz = cosf(half_theta);
        float sz = sinf(half_theta);
        odom_msg.pose.pose.orientation.x = 0.0f;
        odom_msg.pose.pose.orientation.y = 0.0f;
        odom_msg.pose.pose.orientation.z = sz;
        odom_msg.pose.pose.orientation.w = cz;

        // 线速度/角速度（车体坐标系）
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = 0.0f;
        odom_msg.twist.twist.linear.z = 0.0f;
        odom_msg.twist.twist.angular.x = 0.0f;
        odom_msg.twist.twist.angular.y = 0.0f;
        odom_msg.twist.twist.angular.z = wz;

        RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
        // ------------------ 里程计处理结束 ------------------------

        // ------------------ /cmd_vel 看门狗：超时自动刹车 ---------
        // 例如 300ms 内没有收到新的 /cmd_vel，就认为上位机失联，主动停车
        const int64_t CMD_VEL_TIMEOUT_US = 300000; // 0.3s
        if (last_cmd_time_us != 0) {
            int64_t no_cmd_duration = now_us - last_cmd_time_us;
            if (no_cmd_duration > CMD_VEL_TIMEOUT_US) {
                // 超时未收到新指令，主动停车（刹车停止）
                Motion_Stop(STOP_BRAKE);
            }
        }
        // ------------------ 看门狗检查结束 ------------------------

        usleep(10000); // 10ms 轻量延时，避免占满 CPU
    }

    // 清理资源 (正常情况下不会执行到这里)
    RCCHECK(rcl_publisher_fini(&log_publisher, &node)); 
    RCCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rclc_support_fini(&support));
    vTaskDelete(NULL);
}

// ============================================================================
// Wi-Fi 事件处理函数：负责连接 AP 与响应断线重连
// ============================================================================
// 说明：
//   - STA_START        : 启动后立即尝试连接 AP
//   - STA_DISCONNECTED : 断开后自动重连
//   - STA_GOT_IP       : 获取 IP 后启动 micro_ros_task
// ============================================================================
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
        // 连接成功后启动 micro-ROS 主任务
        xTaskCreate(micro_ros_task, "micro_ros_task", 16000, NULL, 5, NULL);
    }
}

// ============================================================================
// Wi-Fi STA 模式初始化：配置为客户端并连接到指定 AP
// ============================================================================
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

// ============================================================================
// app_main：ESP-IDF 程序入口
// ============================================================================
// 流程：
//   1. 初始化 NVS（用于 Wi-Fi 等模块存储）
//   2. 初始化小车底层硬件（电机、编码器、PID 等）
//   3. 启动 Wi-Fi，并在拿到 IP 后启动 micro-ROS 任务
// ============================================================================
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
