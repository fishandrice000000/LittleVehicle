#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdarg.h> 
#include <math.h> 

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/init_options.h>

#include "geometry_msgs/msg/twist.h"
#include "rcl_interfaces/msg/log.h" 
#include "nav_msgs/msg/odometry.h"
#include <sensor_msgs/msg/imu.h>

#include "car_motion.h"
#include "icm42670p.h"
#include "pid_ctrl.h"

// ================================================================= //
// ==================== 用户需要修改的配置 ======================== //
// ================================================================= //

// WiFi网络配置
#define WIFI_SSID "aaa"
#define WIFI_PASSWORD "12345678"

// microROS Agent配置 (运行Agent的电脑IP)
#define MICRO_ROS_AGENT_IP "10.37.89.86"
#define MICRO_ROS_AGENT_PORT "8090"

// micro-ROS连接重试配置
#define MAX_RETRY_ATTEMPTS   5
#define RETRY_DELAY_MS     2000

static const char *TAG = "MAIN";

// ===== micro-ROS 错误处理宏 =====
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Aborting.", __LINE__, (int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ESP_LOGE(TAG, "Failed status on line %d: %d. Continuing.", __LINE__, (int)temp_rc);}}

// ===== ROS 句柄与消息对象（全局） =====
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

rcl_publisher_t log_publisher;     
rcl_interfaces__msg__Log log_msg;

rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

static float odom_x = 0.0f;
static float odom_y = 0.0f;
static float odom_theta = 0.0f;

static int64_t last_cmd_time_us = 0;

// ===== 角速度 PID 控制变量 =====
static float         cmd_vx = 0.0f, cmd_wz = 0.0f; // 最新 /cmd_vel 目标
static float         gyro_z_bias_global = 0.0f;     // IMU 零偏（校准时写入）
static pid_ctrl_block_handle_t wz_pid = NULL;       // 角速度 PID 句柄

void publish_log(uint8_t level, const char *file, const char *function, int line, const char *name, const char *format, ...)
{
    if (!rcl_node_is_valid(&node)) {
        return;
    }

    int64_t stamp_nanos = 0;
    if (rmw_uros_epoch_synchronized()) {
        stamp_nanos = rmw_uros_epoch_nanos();
    }
    log_msg.stamp.sec = stamp_nanos / 1000000000;
    log_msg.stamp.nanosec = stamp_nanos % 1000000000;
    
    log_msg.level = level;
    strncpy(log_msg.name.data, name, log_msg.name.capacity - 1);
    log_msg.name.size = strlen(log_msg.name.data);
    strncpy(log_msg.file.data, file, log_msg.file.capacity - 1);
    log_msg.file.size = strlen(log_msg.file.data);
    strncpy(log_msg.function.data, function, log_msg.function.capacity - 1);
    log_msg.function.size = strlen(log_msg.function.data);
    log_msg.line = line;

    va_list args;
    va_start(args, format);
    vsnprintf(log_msg.msg.data, log_msg.msg.capacity - 1, format, args);
    log_msg.msg.size = strlen(log_msg.msg.data);
    va_end(args);
    
    RCSOFTCHECK(rcl_publish(&log_publisher, &log_msg, NULL));
}

#define PUBLISH_LOG_INFO(NODE_NAME, FORMAT, ...) \
    publish_log(rcl_interfaces__msg__Log__INFO, __FILE__, __func__, __LINE__, NODE_NAME, FORMAT, ##__VA_ARGS__)

void cmd_vel_subscription_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msgin;

    cmd_vx = twist_msg->linear.x;
    cmd_wz = twist_msg->angular.z;
    last_cmd_time_us = esp_timer_get_time();

    PUBLISH_LOG_INFO(rcl_node_get_name(&node), "收到速度指令 Vx: %.2f, Wz: %.2f", cmd_vx, cmd_wz);
}

// ============================================================================
// 角速度 PID 控制任务 (100Hz, 独立 FreeRTOS 任务)
// ============================================================================
// 参照 motor.c 中 Motor_Task 的模式:
//   - vTaskDelayUntil 严格 100Hz
//   - 看门狗超时 → Motion_Stop
//   - 用 IMU gyro z 做反馈, PID 修正 Wz 后再调 Motion_Ctrl
// ============================================================================
static void wz_pid_task(void *arg)
{
    ESP_LOGI(TAG, "wz_pid_task started.");
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));  // 100Hz

        int64_t now = esp_timer_get_time();

        // 看门狗: 100ms 没收到 /cmd_vel → 刹车
        if (last_cmd_time_us != 0 && (now - last_cmd_time_us) > 100000) {
            Motion_Stop(STOP_BRAKE);
            cmd_vx = 0.0f;
            cmd_wz = 0.0f;
            continue;
        }

        float vx_out = cmd_vx;
        float wz_out = cmd_wz;

        // 角速度 PID: 旋转时用 IMU 反馈修正
        if (cmd_wz != 0.0f && wz_pid != NULL && Icm42670p_Start_OK() > 0) {
            float gyro[3];
            Icm42670p_Get_Gyro_dps(gyro);
            float actual_wz = gyro[2] - gyro_z_bias_global;
            float error = cmd_wz - actual_wz;
            float pid_out = 0.0f;
            pid_compute(wz_pid, error, &pid_out);
            wz_out += pid_out;
        }

        Motion_Ctrl(vx_out, 0, wz_out);
    }

    vTaskDelete(NULL);
}

void micro_ros_task(void *arg)
{
    ESP_LOGI(TAG, "micro-ROS 任务已启动.");
    
    bool is_initialized = false;
    for (int attempts = 1; attempts <= MAX_RETRY_ATTEMPTS; attempts++) {
        ESP_LOGI(TAG, "micro-ROS 初始化尝试 #%d/%d", attempts, MAX_RETRY_ATTEMPTS);
        
        do {
            allocator = rcl_get_default_allocator();
            rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
            if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) break;

            rmw_init_options_t* rmw_init_options = rcl_init_options_get_rmw_init_options(&init_options);
            if (rmw_uros_options_set_udp_address(MICRO_ROS_AGENT_IP, MICRO_ROS_AGENT_PORT, rmw_init_options) != RMW_RET_OK) break;

            if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) break;

            if (rmw_uros_sync_session(1000) != RMW_RET_OK) {
                ESP_LOGE(TAG, "与Agent的时间同步失败。");
                break;
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

            // odom 协方差矩阵 (EKF 强制要求非零)
            memset(odom_msg.pose.covariance, 0, sizeof(odom_msg.pose.covariance));
            memset(odom_msg.twist.covariance, 0, sizeof(odom_msg.twist.covariance));
            for (int i = 0; i < 6; i++) {
                odom_msg.pose.covariance[i*6+i] = 0.01;
                odom_msg.twist.covariance[i*6+i] = 0.01;
            }

            // 初始化 /imu publisher
            if (rclc_publisher_init_default(
                    &imu_publisher,
                    &node,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                    "/imu") != RCL_RET_OK) break;

            static char imu_frame_id[16];
            imu_msg.header.frame_id.data = imu_frame_id;
            imu_msg.header.frame_id.capacity = sizeof(imu_frame_id);
            strncpy(imu_msg.header.frame_id.data, "imu_link", imu_msg.header.frame_id.capacity - 1);
            imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);

            // IMU 协方差矩阵 (EKF 强制要求非零)
            memset(imu_msg.orientation_covariance, 0, sizeof(imu_msg.orientation_covariance));
            memset(imu_msg.angular_velocity_covariance, 0, sizeof(imu_msg.angular_velocity_covariance));
            memset(imu_msg.linear_acceleration_covariance, 0, sizeof(imu_msg.linear_acceleration_covariance));
            for (int i = 0; i < 3; i++) {
                imu_msg.orientation_covariance[i*3+i] = 0.001;
                imu_msg.angular_velocity_covariance[i*3+i] = 0.001;
                imu_msg.linear_acceleration_covariance[i*3+i] = 0.01;
            }

            if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) break;
            
            if (rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_subscription_callback, ON_NEW_DATA) != RCL_RET_OK) break;

            is_initialized = true;

        } while (0);

        if (is_initialized) {
            ESP_LOGI(TAG, "micro-ROS 初始化成功。");
            break;
        }

        ESP_LOGE(TAG, "micro-ROS 初始化失败，正在清理...");
        RCSOFTCHECK(rclc_executor_fini(&executor));
        RCSOFTCHECK(rcl_publisher_fini(&log_publisher, &node));
        RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
        RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));
        RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
        RCSOFTCHECK(rcl_node_fini(&node));
        RCSOFTCHECK(rclc_support_fini(&support));
        
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
    }

    if (!is_initialized) {
        ESP_LOGE(TAG, "无法初始化 micro-ROS。任务中止。");
        vTaskDelete(NULL);
    }
    
    PUBLISH_LOG_INFO(rcl_node_get_name(&node), "节点已创建，执行器已启动。");

    // ===== IMU 陀螺仪零偏校准 (小车此时应当静止) =====
    {
        int calib_samples = 0;
        ESP_LOGI(TAG, "正在校准陀螺仪零偏，请保持小车静止...");
        while (Icm42670p_Start_OK() <= 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        float gyro_dps[3];
        for (int i = 0; i < 200; i++) {
            Icm42670p_Get_Gyro_dps(gyro_dps);
            gyro_z_bias_global += gyro_dps[2];
            calib_samples++;
            usleep(10000);
        }
        gyro_z_bias_global /= (float)calib_samples;
        ESP_LOGI(TAG, "陀螺仪 Z 轴零偏: %.4f rad/s (%.2f °/s)",
                 gyro_z_bias_global, gyro_z_bias_global * 180.0f / M_PI);
    }

    // ===== 创建角速度 PID 任务 =====
    {
        pid_ctrl_parameter_t wz_pid_param = {
            .kp = 0.5, .ki = 0.05, .kd = 0.0,
            .max_output = 2.0, .min_output = -2.0,
            .max_integral = 1.0, .min_integral = -1.0,
            .cal_type = PID_CAL_TYPE_INCREMENTAL,
        };
        pid_ctrl_config_t wz_pid_cfg = { .init_param = wz_pid_param };
        pid_new_control_block(&wz_pid_cfg, &wz_pid);
        xTaskCreate(wz_pid_task, "wz_pid_task", 4*1024, NULL, 5, NULL);
        ESP_LOGI(TAG, "角速度 PID 任务已创建.");
    }

    int64_t last_odom_time_us = esp_timer_get_time();

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        // -------------------- 里程计计算与发布 --------------------
        int64_t now_us = esp_timer_get_time();
        float dt = (now_us - last_odom_time_us) / 1000000.0f;
        if (dt < 0.0f) dt = 0.0f;
        if (dt > 1.0f) dt = 1.0f;
        last_odom_time_us = now_us;

        // 获取车体速度
        // car_motion.c 中将四轮速度融合为车体中心速度，对上层透明。
        car_motion_t car;
        Motion_Get_Speed(&car);

        float vx = car.Vx;     
        float wz = car.Wz;     

        // 差速模型的里程计积分
        odom_theta += wz * dt;

        float cos_t = cosf(odom_theta);
        float sin_t = sinf(odom_theta);
        odom_x += vx * cos_t * dt;
        odom_y += vx * sin_t * dt;

        int64_t stamp_nanos = 0;
        if (rmw_uros_epoch_synchronized()) {
            stamp_nanos = rmw_uros_epoch_nanos();
        } else {
            stamp_nanos = now_us * 1000;
        }
        odom_msg.header.stamp.sec = stamp_nanos / 1000000000;
        odom_msg.header.stamp.nanosec = stamp_nanos % 1000000000;

        odom_msg.pose.pose.position.x = odom_x;
        odom_msg.pose.pose.position.y = odom_y;
        odom_msg.pose.pose.position.z = 0.0f;

        float half_theta = odom_theta * 0.5f;
        float cz = cosf(half_theta);
        float sz = sinf(half_theta);
        odom_msg.pose.pose.orientation.x = 0.0f;
        odom_msg.pose.pose.orientation.y = 0.0f;
        odom_msg.pose.pose.orientation.z = sz;
        odom_msg.pose.pose.orientation.w = cz;

        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = 0.0f;
        odom_msg.twist.twist.linear.z = 0.0f;
        odom_msg.twist.twist.angular.x = 0.0f;
        odom_msg.twist.twist.angular.y = 0.0f;
        odom_msg.twist.twist.angular.z = wz;

        RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

        // -------------------- IMU 数据发布 --------------------
        if (Icm42670p_Start_OK() > 0)
        {
            float gyro_dps[3], accel_g[3];
            Icm42670p_Get_Gyro_dps(gyro_dps);
            Icm42670p_Get_Accel_g(accel_g);

            imu_msg.header.stamp.sec = odom_msg.header.stamp.sec;
            imu_msg.header.stamp.nanosec = odom_msg.header.stamp.nanosec;

            imu_msg.angular_velocity.x = gyro_dps[0];
            imu_msg.angular_velocity.y = gyro_dps[1];
            imu_msg.angular_velocity.z = gyro_dps[2] - gyro_z_bias_global;

            imu_msg.linear_acceleration.x = accel_g[0];
            imu_msg.linear_acceleration.y = accel_g[1];
            imu_msg.linear_acceleration.z = accel_g[2];

            RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
        }

        usleep(10000); 
    }

    RCCHECK(rcl_publisher_fini(&log_publisher, &node));
    RCCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCCHECK(rcl_publisher_fini(&imu_publisher, &node));
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rclc_support_fini(&support));
    vTaskDelete(NULL);
}

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

    ESP_LOGI(TAG, "正在初始化IMU...");
    Icm42670p_Init();
    ESP_LOGI(TAG, "IMU初始化完成.");

    ESP_LOGI(TAG, "正在连接WiFi...");
    wifi_init_sta();
}
