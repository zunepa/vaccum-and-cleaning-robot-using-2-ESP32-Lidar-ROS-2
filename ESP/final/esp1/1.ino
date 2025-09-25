#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Encoder.h>
#include <math.h>
#include <PID_v1.h>

// --- THÔNG SỐ VẬT LÝ ---
const float WHEEL_RADIUS = 0.01649;
const float WHEEL_BASE = 0.236;
const int ENCODER_PPR = 4200;
const float TICKS_PER_METER = ENCODER_PPR / (2.0 * PI * WHEEL_RADIUS);
const float GYRO_DEADZONE = 0.02; // <<< THÊM DÒNG NÀY: Ngưỡng khử trôi cho Gyro (rad/s) >>>


// --- KHAI BÁO CHÂN ---
const int MOTOR_R_IN1 = 12, MOTOR_R_IN2 = 14, MOTOR_R_ENA = 13;
const int ENCODER_R_C1 = 33, ENCODER_R_C2 = 32;
const int MOTOR_L_IN3 = 27, MOTOR_L_IN4 = 26, MOTOR_L_ENB = 25;
const int ENCODER_L_C1 = 34, ENCODER_L_C2 = 35;

// --- CÀI ĐẶT PWM ---
const int PWM_CHANNEL_R = 0, PWM_CHANNEL_L = 1;
const int PWM_FREQUENCY = 5000, PWM_RESOLUTION = 8;

// --- BIẾN TOÀN CỤC ---
ESP32Encoder encoderRight, encoderLeft;
Adafruit_MPU6050 mpu;

// <<< CÁC BIẾN CHO BỘ LỌC BÙ VÀ ODOMETRY >>>
float gyroZ_bias = 0.0;
float fused_yaw = 0.0; // Góc yaw cuối cùng sau khi kết hợp dữ liệu
unsigned long lastOdomTime = 0;
long last_right_counts_odom = 0, last_left_counts_odom = 0;
// <<< KẾT THÚC >>>

unsigned long lastPIDTime = 0, lastCmdTime = 0;
const int PID_SAMPLE_TIME_MS = 20, CMD_TIMEOUT_MS = 500;
double Kp_L = 0.045, Ki_L = 0.2, Kd_L = 0.0001;
double Kp_R = 0.07, Ki_R = 0.2, Kd_R = 0.0001;
float target_linear_x = 0.0, target_angular_z = 0.0;
double target_ticks_per_sec_L = 0, actual_ticks_per_sec_L = 0, output_pwm_L = 0;
double target_ticks_per_sec_R = 0, actual_ticks_per_sec_R = 0, output_pwm_R = 0;
long prev_ticks_L = 0, prev_ticks_R = 0;
PID pidLeft(&actual_ticks_per_sec_L, &output_pwm_L, &target_ticks_per_sec_L, Kp_L, Ki_L, Kd_L, DIRECT);
PID pidRight(&actual_ticks_per_sec_R, &output_pwm_R, &target_ticks_per_sec_R, Kp_R, Ki_R, Kd_R, DIRECT);

// --- CÁC HÀM ---
void handleCommands() {
  if (Serial2.available() > 0) {
    String command = Serial2.readStringUntil('\n');
    command.trim();
    if (command.startsWith("V:")) {
      lastCmdTime = millis();
      int first_comma = command.indexOf(',');
      if (first_comma != -1) {
        String linear_str = command.substring(2, first_comma);
        String angular_str = command.substring(first_comma + 1);
        target_linear_x = linear_str.toFloat();
        target_angular_z = angular_str.toFloat();
      }
    }
  }
}

void controlMotors() {
    if (millis() - lastPIDTime >= PID_SAMPLE_TIME_MS) {
        lastPIDTime = millis();

        double target_speed_R_mps = target_linear_x + (target_angular_z * WHEEL_BASE / 2.0);
        double target_speed_L_mps = target_linear_x - (target_angular_z * WHEEL_BASE / 2.0);
        target_ticks_per_sec_R = target_speed_R_mps * TICKS_PER_METER;
        target_ticks_per_sec_L = target_speed_L_mps * TICKS_PER_METER;
        
        if (millis() - lastCmdTime > CMD_TIMEOUT_MS) {
            target_linear_x = 0.0;
            target_angular_z = 0.0;
        }

        if (target_linear_x == 0.0 && target_angular_z == 0.0) {
            ledcWrite(PWM_CHANNEL_L, 0);
            ledcWrite(PWM_CHANNEL_R, 0);
            digitalWrite(MOTOR_L_IN3, LOW); digitalWrite(MOTOR_L_IN4, LOW);
            digitalWrite(MOTOR_R_IN1, LOW); digitalWrite(MOTOR_R_IN2, LOW);
            return;
        }

        long current_ticks_L = encoderLeft.getCount();
        long current_ticks_R = encoderRight.getCount();
        actual_ticks_per_sec_L = (double)(current_ticks_L - prev_ticks_L) * 1000.0 / PID_SAMPLE_TIME_MS;
        actual_ticks_per_sec_R = (double)(current_ticks_R - prev_ticks_R) * 1000.0 / PID_SAMPLE_TIME_MS;
        prev_ticks_L = current_ticks_L;
        prev_ticks_R = current_ticks_R;

        pidLeft.Compute();
        pidRight.Compute();

        if (fabs(output_pwm_L) < 20) output_pwm_L = 0;
        if (fabs(output_pwm_R) < 20) output_pwm_R = 0;

        if (output_pwm_L >= 0) {
            digitalWrite(MOTOR_L_IN3, LOW); digitalWrite(MOTOR_L_IN4, HIGH);
        } else {
            digitalWrite(MOTOR_L_IN3, HIGH); digitalWrite(MOTOR_L_IN4, LOW);
        }
        ledcWrite(PWM_CHANNEL_L, fabs(output_pwm_L));

        if (output_pwm_R >= 0) {
            digitalWrite(MOTOR_R_IN1, LOW); digitalWrite(MOTOR_R_IN2, HIGH);
        } else {
            digitalWrite(MOTOR_R_IN1, HIGH); digitalWrite(MOTOR_R_IN2, LOW);
        }
        ledcWrite(PWM_CHANNEL_R, fabs(output_pwm_R));
    }
}

// <<< HÀM LẤY TỐC ĐỘ GÓC TỪ GYRO >>>
// <<< THAY THẾ TOÀN BỘ HÀM CŨ BẰNG HÀM NÀY >>>
float getGyroZRate() {
    sensors_event_t a, g, temp; 
    mpu.getEvent(&a, &g, &temp);
    
    // Tính toán tốc độ góc thô đã hiệu chỉnh bias
    float raw_rate = (g.gyro.z - gyroZ_bias);
     Serial.println(raw_rate);
    // Áp dụng ngưỡng chết (deadzone)
    // Nếu tốc độ quay quá nhỏ, coi như bằng 0 để khử trôi do nhiễu
    if (fabs(raw_rate) < GYRO_DEADZONE) {
        return 0.0;
    }
    
    return raw_rate;
}

// <<< HÀM CHÍNH: TÍNH TOÁN, KẾT HỢP DỮ LIỆU VÀ GỬI ODOMETRY ĐI >>>
void updateAndPublishFusedOdometry() {
    // Gửi dữ liệu ở tần suất 20Hz (50ms)
    if (millis() - lastOdomTime > 50) { 
        unsigned long current_time = millis();
        float dt_sec = (current_time - lastOdomTime) / 1000.0;
        lastOdomTime = current_time;

        // 1. LẤY DỮ LIỆU TỪ ENCODER
        long current_right_counts = encoderRight.getCount();
        long current_left_counts = encoderLeft.getCount();
        long delta_right_counts = current_right_counts - last_right_counts_odom;
        long delta_left_counts = current_left_counts - last_left_counts_odom;
        last_right_counts_odom = current_right_counts;
        last_left_counts_odom = current_left_counts;
        
        float dist_right_m = delta_right_counts / TICKS_PER_METER;
        float dist_left_m = delta_left_counts / TICKS_PER_METER;

        float linear_velocity_x = (dist_right_m + dist_left_m) / (2.0 * dt_sec);
        float angular_velocity_odom = (dist_right_m - dist_left_m) / (WHEEL_BASE * dt_sec);

        // 2. LẤY DỮ LIỆU TỪ GYROSCOPE
        float angular_velocity_gyro = getGyroZRate();

        // 3. ÁP DỤNG BỘ LỌC BÙ (COMPLEMENTARY FILTER)
        // Alpha quyết định mức độ tin tưởng vào gyro (phản ứng nhanh) vs encoder (ổn định dài hạn)
        const float alpha = 0.98; 
        float combined_angular_velocity = alpha * angular_velocity_gyro + (1.0 - alpha) * angular_velocity_odom;

        // 4. TÍCH HỢP ĐỂ CÓ GÓC YAW CUỐI CÙNG
        fused_yaw += combined_angular_velocity * dt_sec;

        // Chuẩn hóa góc trong khoảng -PI đến PI
        if (fused_yaw > PI) { fused_yaw -= 2 * PI; } 
        else if (fused_yaw < -PI) { fused_yaw += 2 * PI; }

        // 5. GỬI DỮ LIỆU ĐI
        // Định dạng mới: "O:vx,wz,yaw" (vận tốc dài, vận tốc góc, góc quay)
        String odomData = "O:" + String(linear_velocity_x, 4) + "," + String(combined_angular_velocity, 4) + "," + String(fused_yaw, 4) + "\n";
        Serial2.print(odomData);
        //Serial.println(odomData); // Dùng dòng này để debug trên Serial Monitor
    }
}

void setupMotors() {
    pinMode(MOTOR_R_IN1, OUTPUT); pinMode(MOTOR_R_IN2, OUTPUT);
    pinMode(MOTOR_L_IN3, OUTPUT); pinMode(MOTOR_L_IN4, OUTPUT);
    ledcSetup(PWM_CHANNEL_R, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_L, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_R_ENA, PWM_CHANNEL_R);
    ledcAttachPin(MOTOR_L_ENB, PWM_CHANNEL_L);
}

void setupEncoders() {
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoderLeft.attachFullQuad(ENCODER_L_C1, ENCODER_L_C2);
    encoderRight.attachFullQuad(ENCODER_R_C2, ENCODER_R_C1); 
    encoderLeft.clearCount();
    encoderRight.clearCount();
}

// <<< KÍCH HOẠT LẠI HÀM CÀI ĐẶT VÀ HIỆU CHỈNH MPU6050 >>>
void setupMPU6050() {
    if (!mpu.begin()) { 
      Serial.println("Lỗi: Không tìm thấy MPU6050!");
      while (1) { delay(10); } 
    }
    Serial.println("Đang hiệu chỉnh Gyro, vui lòng giữ yên robot...");
    float total_error = 0;
    for (int i = 0; i < 2000; i++) {
        sensors_event_t a, g, temp; 
        mpu.getEvent(&a, &g, &temp); 
        total_error += g.gyro.z; 
        delay(1);
    }
    gyroZ_bias = total_error / 2000.0;
    Serial.println("Hiệu chỉnh Gyro hoàn tất!");
    Serial.print("Gyro Z Bias: ");
    Serial.println(gyroZ_bias);
}

// --- SETUP & LOOP CHÍNH ---
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 19, 18);
  Wire.begin();
  
  setupMotors();
  setupEncoders();
  setupMPU6050(); // <<< KÍCH HOẠT LẠI MPU6050

  last_right_counts_odom = encoderRight.getCount();
  last_left_counts_odom = encoderLeft.getCount();
  prev_ticks_L = encoderLeft.getCount();
  prev_ticks_R = encoderRight.getCount();
  lastCmdTime = millis();

  pidLeft.SetMode(AUTOMATIC);
  pidRight.SetMode(AUTOMATIC);
  pidLeft.SetSampleTime(PID_SAMPLE_TIME_MS);
  pidRight.SetSampleTime(PID_SAMPLE_TIME_MS);
  pidLeft.SetOutputLimits(-255, 255);
  pidRight.SetOutputLimits(-255, 255);

  Serial.println("ESP1 Controller - Fused Odometry - Ready.");
}

void loop() {
  handleCommands();
  controlMotors();
  
  // <<< THAY THẾ CÁC HÀM CŨ BẰNG HÀM MỚI DUY NHẤT NÀY >>>
  updateAndPublishFusedOdometry();
}
