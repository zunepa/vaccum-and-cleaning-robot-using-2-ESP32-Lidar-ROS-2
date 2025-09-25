// --- PHẦN 1: CÁC THƯ VIỆN CẦN THIẾT ---
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <WiFi.h>
#include "lds_all_models.h"

// --- PHẦN 2: CẤU HÌNH ---
char ssid[] = "";
char password[] = "";
char agent_ip[] = "";
const uint16_t agent_port = 1605;

// Lidar & Giao tiếp ESP1
const uint8_t LIDAR_GPIO_RX = 16, LIDAR_GPIO_TX = 17, LIDAR_GPIO_PWM = 4;
const uint32_t LIDAR_PWM_FREQ = 10000;
const uint8_t LIDAR_PWM_BITS = 8;
const uint8_t LIDAR_PWM_CHANNEL = 2;
const uint8_t ESP1_UART_RX = 33, ESP1_UART_TX = 32;

#define LED_PIN 2
// --- PHẦN 3: CÁC BIẾN TOÀN CỤC ---
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t scan_publisher;
rcl_publisher_t odom_publisher;
rcl_timer_t lidar_loop_timer;
rcl_timer_t odom_timer;
rcl_subscription_t cmd_vel_subscriber;

sensor_msgs__msg__LaserScan scan_msg;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

HardwareSerial LidarSerial(2);
LDS* lidar;
HardwareSerial Esp1Serial(1);

// Biến lưu trữ vị trí của robot
float x_pos = 0.0, y_pos = 0.0, theta_pos = 0.0;
// Biến để tính dt chính xác
unsigned long last_odom_update_time = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ Serial.printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); ESP.restart();}}

// --- PHẦN 4: KHAI BÁO HÀM ---
void lidar_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void odom_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void cmd_vel_callback(const void *msgin);
void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed);
int lidar_serial_read_callback();
size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length);
void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin);
void connect_wifi();
void connect_agent();
void initialize_esp1_communication();
void create_ros_entities();
void configure_scan_message();
void configure_odom_message();
void initialize_lidar_hardware();


// --- PHẦN 5: ĐỊNH NGHĨA HÀM ---
void connect_wifi() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
}

void connect_agent() {
    set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
    Serial.println("Pinging agent...");
    while (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
        Serial.print(".");
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(500);
    }
    digitalWrite(LED_PIN, HIGH);
    Serial.println("Agent successfully connected.");
}

void initialize_esp1_communication() {
    Serial.println("Initializing communication with ESP1...");
    Esp1Serial.begin(115200, SERIAL_8N1, ESP1_UART_RX, ESP1_UART_TX);
}

void create_ros_entities() {
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_robot_node", "", &support));
    RCCHECK(rclc_publisher_init_default(&scan_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan"));
    RCCHECK(rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));
    Serial.println("Creating subscriber for /cmd_vel...");
    RCCHECK(rclc_subscription_init_default(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    const unsigned int lidar_timer_period = 10;
    RCCHECK(rclc_timer_init_default(&lidar_loop_timer, &support, RCL_MS_TO_NS(lidar_timer_period), lidar_timer_callback));
    const unsigned int odom_timer_period = 50;
    RCCHECK(rclc_timer_init_default(&odom_timer, &support, RCL_MS_TO_NS(odom_timer_period), odom_timer_callback));
    Serial.println("Creating executor...");
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &lidar_loop_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
}

void configure_scan_message() {
    Serial.println("Configuring LaserScan message...");
    scan_msg.header.frame_id.data = (char *) "laser_frame";
    scan_msg.header.frame_id.size = strlen(scan_msg.header.frame_id.data);
    scan_msg.angle_min = 0.0;
    scan_msg.angle_max = 2.0 * PI;
    scan_msg.angle_increment = (2.0 * PI) / 360.0;
    scan_msg.scan_time = 0.2;
    scan_msg.range_min = 0.12;
    scan_msg.range_max = 12.0;
    scan_msg.ranges.size = 360;
    scan_msg.ranges.capacity = 360;
    scan_msg.ranges.data = (float*) malloc(scan_msg.ranges.capacity * sizeof(float));
}

void configure_odom_message() {
    Serial.println("Configuring Odometry message...");
    odom_msg.header.frame_id.data = (char *) "odom";
    odom_msg.child_frame_id.data = (char *) "base_link";
    odom_msg.pose.covariance[0] = 0.001;
    odom_msg.pose.covariance[7] = 0.001;
    odom_msg.pose.covariance[14] = 1e9;
    odom_msg.pose.covariance[21] = 1e9;
    odom_msg.pose.covariance[28] = 1e9;
    odom_msg.pose.covariance[35] = 0.01;
    odom_msg.twist.covariance[0] = 0.001;
    odom_msg.twist.covariance[7] = 0.001;
    odom_msg.twist.covariance[35] = 0.01;
}

void initialize_lidar_hardware() {
    Serial.println("Initializing Lidar hardware...");
    lidar = new LDS_RPLIDAR_A1();
    lidar->setSerialWriteCallback(lidar_serial_write_callback);
    lidar->setSerialReadCallback(lidar_serial_read_callback);
    lidar->setMotorPinCallback(lidar_motor_pin_callback);
    LidarSerial.begin(lidar->getSerialBaudRate(), SERIAL_8N1, LIDAR_GPIO_RX, LIDAR_GPIO_TX);
    lidar->init();
    lidar->start();
    lidar->setScanPointCallback(lidar_scan_point_callback);
}

void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    float linear_x = msg->linear.x;
    float angular_z = msg->angular.z;
    String command_string = "V:" + String(linear_x, 2) + "," + String(angular_z, 2) + "\n";
    Esp1Serial.print(command_string);
}

// <<< HÀM NÀY ĐÃ ĐƯỢC SỬA ĐỔI HOÀN TOÀN >>>
void odom_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer == NULL) {
        return;
    }

    if (Esp1Serial.available() > 0) {
        String odom_data_str = Esp1Serial.readStringUntil('\n');

        if (odom_data_str.startsWith("O:")) {
            odom_data_str.replace("O:", "");
            char buffer[50];
            odom_data_str.toCharArray(buffer, 50);
            
            // 1. PHÂN TÍCH CHUỖI 3 GIÁ TRỊ MỚI
            char *p = buffer;
            char *str;
            float linear_vel_x = 0, angular_vel_z = 0, fused_yaw_from_esp1 = 0;
            int i = 0;
            while ((str = strtok_r(p, ",", &p)) != NULL) {
                if (i == 0) linear_vel_x = atof(str);
                if (i == 1) angular_vel_z = atof(str);
                if (i == 2) fused_yaw_from_esp1 = atof(str); // Lấy giá trị yaw mới
                i++;
            }

            // Tính toán khoảng thời gian delta_t
            unsigned long current_time_ms = millis();
            if (last_odom_update_time == 0) {
                last_odom_update_time = current_time_ms;
                return;
            }
            float delta_t = (current_time_ms - last_odom_update_time) / 1000.0;
            last_odom_update_time = current_time_ms;

            // 2. SỬ DỤNG TRỰC TIẾP GÓC YAW TỪ ESP1
            // Không cần tích phân vận tốc góc nữa, ta đã có góc quay chính xác.
            theta_pos = fused_yaw_from_esp1;

            // Tính toán sự thay đổi vị trí x, y dựa trên góc yaw chính xác này
            float delta_x = linear_vel_x * cos(theta_pos) * delta_t;
            float delta_y = linear_vel_x * sin(theta_pos) * delta_t;
            x_pos += delta_x;
            y_pos += delta_y;

            // Cập nhật và xuất bản thông điệp Odometry
            rmw_uros_sync_session(10);
            int64_t current_time_ns = rmw_uros_epoch_nanos();
            odom_msg.header.stamp.sec = current_time_ns / 1000000000;
            odom_msg.header.stamp.nanosec = current_time_ns % 1000000000;

            odom_msg.pose.pose.position.x = x_pos;
            odom_msg.pose.pose.position.y = y_pos;
            odom_msg.pose.pose.orientation.z = sin(theta_pos / 2.0);
            odom_msg.pose.pose.orientation.w = cos(theta_pos / 2.0);

            odom_msg.twist.twist.linear.x = linear_vel_x;
            odom_msg.twist.twist.angular.z = angular_vel_z;

            rcl_publish(&odom_publisher, &odom_msg, NULL);
        }
    }
}

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
    static bool reset_ranges = true;
    if (reset_ranges) {
        for (int i = 0; i < 360; i++) {
            scan_msg.ranges.data[i] = INFINITY;
        }
        reset_ranges = false;
    }
    int index = 360 - (int)(angle_deg + 0.5);
    if (index < 0 || index >= 360) return;

    if (distance_mm <= 0 || distance_mm > 12000) {
        scan_msg.ranges.data[index] = INFINITY;
    } else {
        scan_msg.ranges.data[index] = distance_mm / 1000.0;
    }

    if (scan_completed) {
        rmw_uros_sync_session(10);
        int64_t current_time = rmw_uros_epoch_nanos();
        scan_msg.header.stamp.sec = current_time / 1000000000;
        scan_msg.header.stamp.nanosec = current_time % 1000000000;
        rcl_publish(&scan_publisher, &scan_msg, NULL);
        reset_ranges = true;
    }
}

void lidar_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL && lidar != NULL) {
        lidar->loop();
    }
}

int lidar_serial_read_callback() {
    return LidarSerial.read();
}
size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length) {
    return LidarSerial.write(buffer, length);
}
void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin) {
    int pin = LIDAR_GPIO_PWM;
    if (value <= (float)LDS::DIR_INPUT) {
        if (value == (float)LDS::DIR_OUTPUT_PWM) {
            ledcSetup(LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_PWM_BITS);
            ledcAttachPin(pin, LIDAR_PWM_CHANNEL);
        } else {
            pinMode(pin, (value == (float)LDS::DIR_INPUT) ? INPUT : OUTPUT);
        }
        return;
    }
    if (value < (float)LDS::VALUE_PWM) {
        digitalWrite(pin, (value == (float)LDS::VALUE_HIGH) ? HIGH : LOW);
    } else {
        int pwm_value = ((1 << LIDAR_PWM_BITS) - 1) * value;
        ledcWrite(LIDAR_PWM_CHANNEL, pwm_value);
    }
}

// --- SETUP & LOOP ---
// ---  & LOOP ---
void setup() {
    Serial.begin(115200);
    delay(1000);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    // Bước 1: Kết nối WiFi và Agent như bình thường
    connect_wifi();
    connect_agent();
    
    // Bước 2: Tạo các thực thể ROS
    initialize_esp1_communication();
    create_ros_entities();
    configure_scan_message();
    configure_odom_message();
    initialize_lidar_hardware();

    // --- PHẦN SỬA LỖI ĐỒNG BỘ THỜI GIAN ---
    // Bước 3: Sau khi mọi thứ đã sẵn sàng, thực hiện đồng bộ thời gian
    Serial.println("Attempting to sync time with agent...");
    bool time_synced = false;
    // Thử đồng bộ trong tối đa 5 giây
    for (int i = 0; i < 50; i++) {
        rmw_ret_t sync_result = rmw_uros_sync_session(100);
        if (sync_result == RMW_RET_OK) {
            int64_t synced_time = rmw_uros_epoch_nanos();
            if (synced_time > 1000000000) {
                 time_synced = true;
                 Serial.printf("Time successfully synchronized! Current epoch time: %lld ns\n", synced_time);
                 break;
            }
        }
        delay(100);
    }

    if (!time_synced) {
        Serial.println("Failed to sync time. Restarting...");
        ESP.restart();
    }
    // --- KẾT THÚC PHẦN SỬA LỖI ---

    last_odom_update_time = millis();
    Serial.println("--- Setup finished successfully! All systems go. ---");
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    delay(10);
}
