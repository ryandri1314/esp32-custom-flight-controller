#include "baro_data.h"
#include "rx_data.h"
#include "imu_data.h"
#include "fusion_math.h"

#include "esp_timer.h"
volatile bool loop_flag = false;

Imu_Data imu_data;

RX_Data rx_data;
// Baro_Data baro_data;

uint16_t receiver[10] = {0};
const uint8_t motor_pin[4] = {1, 0, 3, 4};
const uint8_t pwm_channel[4] = {0, 1, 2, 3};
const uint16_t max_duty_cycle = 650;
const uint8_t resolution = 10;
const uint16_t freq = 16000;

static float ax_f = 0, ay_f = 0, az_f = 0;
float fc = 75.0f;
uint64_t loop_count = 0;
float throttle = 0.0f;
float pwm_vals[4] = {0};

void IRAM_ATTR control_callback(void* arg) {
  loop_flag = true;
}

esp_timer_handle_t control_timer;
uint32_t prev_cycle = 0;

void mixing_pwm(float throttle, float roll, float pitch, float yaw, float* pwm_vals) {
  pwm_vals[0] = (throttle + roll - pitch + yaw);
  pwm_vals[1] = (throttle - roll - pitch - yaw);
  pwm_vals[2] = (throttle - roll + pitch + yaw);
  pwm_vals[3] = (throttle + roll + pitch - yaw);

  float min_pwm = pwm_vals[0];
  float max_pwm = pwm_vals[0];

  for (uint8_t i = 1; i < 4; ++i) {
    if (min_pwm > pwm_vals[i]) min_pwm = pwm_vals[i];
    if (max_pwm < pwm_vals[i]) max_pwm = pwm_vals[i];
  }

  if (max_pwm > 1.0f) {
    float scale = 1.0f / max_pwm;
    for (uint8_t i = 0; i < 4; i++) {
        pwm_vals[i] *= scale;
    }
  }

  for (uint8_t i = 0; i < 4; i++) {
    if (pwm_vals[i] < 0) pwm_vals[i] = 0;
  }

  for (uint8_t i = 0; i < 4; ++i) {
    pwm_vals[i] *= max_duty_cycle;
  }

  ledcWrite(motor_pin[0], pwm_vals[0]);
  ledcWrite(motor_pin[1], pwm_vals[1]);
  ledcWrite(motor_pin[2], pwm_vals[2]);
  ledcWrite(motor_pin[3], pwm_vals[3]);
}

float dt_loop;

const float Kp_pitch_angle = 1.0f;
const float Ki_pitch_angle = 0.0f;
const float Kd_pitch_angle = 0.0f;

const float Kp_roll_angle = 1.0f;
const float Ki_roll_angle = 0.0f;
const float Kd_roll_angle = 0.0f;

float error_pitch_angle = 0.0f;
float error_roll_angle = 0.0f;
float error_yaw_angle = 0.0f;

float target_pitch_angle = 0.0f;
float target_roll_angle = 0.0f;
float target_yaw_angle = 0.0f;

float target_pitch_rate = 0.0f;
float target_roll_rate = 0.0f;
float target_yaw_rate = 0.0f;

const float Kp_pitch_rate = 0.5f;
const float Ki_pitch_rate = 0.0f;
const float Kd_pitch_rate = 0.0f;

const float Kp_roll_rate = 0.5f;
const float Ki_roll_rate = 0.0f;
const float Kd_roll_rate = 0.0f;

const float Kp_yaw_rate = 0.2f;
const float Ki_yaw_rate = 0.0f;
const float Kd_yaw_rate = 0.0f;

float error_pitch_rate = 0.0f;
float error_roll_rate = 0.0f;
float error_yaw_rate = 0.0f;

float error_pitch_rate_last = 0.0f;
float error_roll_rate_last = 0.0f;
float error_yaw_rate_last = 0.0f;

float error_pitch_rate_sum = 0.0f;
float error_roll_rate_sum = 0.0f;
float error_yaw_rate_sum = 0.0f;

float PID_pitch_rate = 0.0f;
float PID_roll_rate = 0.0f;
float PID_yaw_rate = 0.0f;

#define MAX_PITCH_ANGLE 25.0f
#define MAX_ROLL_ANGLE 25.0f
#define MAX_YAW_ANGLE 15.0f
 
#define MAX_PITCH_RATE (200.0f * 0.01745329251f)
#define MAX_ROLL_RATE (200.0f * 0.01745329251f)
#define MAX_YAW_RATE (120.0f * 0.01745329251f)

bool arm_flag = 0;

const float D_FILTER_CUTOFF = 90.0f;

float d_roll_f = 0.0f, d_pitch_f = 0.0f, d_yaw_f = 0.0f;

void PID_angle(uint16_t roll_sticks, uint16_t pitch_sticks, uint16_t yaw_sticks, float roll, float pitch, float yaw) {
  // Pitch
  float pitch_input = (pitch_sticks - 1500) / 500.0f;
  target_pitch_angle = pitch_input * MAX_PITCH_ANGLE;
  error_pitch_angle = target_pitch_angle - pitch;
  target_pitch_rate = Kp_pitch_angle * error_pitch_angle * 0.01745329251f;

  // Roll
  float roll_input = (roll_sticks - 1500) / 500.0f;
  target_roll_angle = roll_input * MAX_ROLL_ANGLE;
  error_roll_angle = target_roll_angle - roll;
  target_roll_rate = Kp_roll_angle * error_roll_angle * 0.01745329251f;

  // Yaw
  float yaw_input = (yaw_sticks - 1500) / 500.0f;
  target_yaw_rate = yaw_input * MAX_YAW_RATE;
}

void PID_rate(float target_roll_rate, float target_pitch_rate, float target_yaw_rate, float gx, float gy, float gz, float dt) {
  if (dt <= 0) {
    return;
  }

  float alpha = (2.0f * PI * D_FILTER_CUTOFF * dt) / (1.0f + 2.0f * PI * D_FILTER_CUTOFF * dt);

  // Roll
  error_roll_rate = target_roll_rate - gx;
  error_roll_rate_sum += (error_roll_rate * dt);
  error_roll_rate_sum = constrain(error_roll_rate_sum, -100, 100);
  float d_roll = (error_roll_rate - error_roll_rate_last) / dt;
  d_roll_f += alpha * (d_roll - d_roll_f);
  PID_roll_rate = (Kp_roll_rate * error_roll_rate) + (Ki_roll_rate * error_roll_rate_sum) + (Kd_roll_rate * d_roll_f);
  PID_roll_rate = constrain(PID_roll_rate, -MAX_ROLL_RATE, MAX_ROLL_RATE);
  error_roll_rate_last = error_roll_rate;

  // Pitch
  error_pitch_rate = target_pitch_rate - gy;
  error_pitch_rate_sum += (error_pitch_rate * dt);
  error_pitch_rate_sum = constrain(error_pitch_rate_sum, -100, 100);
  float d_pitch = (error_pitch_rate - error_pitch_rate_last) / dt;
  d_pitch_f += alpha * (d_pitch - d_pitch_f);
  PID_pitch_rate = (Kp_pitch_rate * error_pitch_rate) + (Ki_pitch_rate * error_pitch_rate_sum) + (Kd_pitch_rate * d_pitch_f);
  PID_pitch_rate = constrain(PID_pitch_rate, -MAX_PITCH_RATE, MAX_PITCH_RATE);
  error_pitch_rate_last = error_pitch_rate;

  // Yaw
  error_yaw_rate = target_yaw_rate - gz;
  error_yaw_rate_sum += (error_yaw_rate * dt);
  error_yaw_rate_sum = constrain(error_yaw_rate_sum, -100, 100);
  float d_yaw = (error_yaw_rate - error_yaw_rate_last) / dt;
  d_yaw_f += alpha * (d_yaw - d_yaw_f);
  PID_yaw_rate = (Kp_yaw_rate * error_yaw_rate) + (Ki_yaw_rate * error_yaw_rate_sum) + (Kd_yaw_rate * d_yaw_f);
  PID_yaw_rate = constrain(PID_yaw_rate, -MAX_YAW_RATE, MAX_YAW_RATE);
  error_yaw_rate_last = error_yaw_rate;
}

float ax, ay, az, gx, gy, gz;
EulerAngles angles;

void setup() {
  Serial.begin(921600);
  imu_data.init();
  imu_data.calibration();
  rx_data.init();
  // baro_data.init();
  // delay(100);
  // baro_data.calibrate();
  delay(500);
  // Serial.print("The offset: ");
  // Serial.println(baro_data.get_offset());
  for (uint8_t i = 0; i < 4; i++) {
    ledcAttach(motor_pin[i], freq, resolution);
  }

  ledcWrite(motor_pin[0], 0);
  ledcWrite(motor_pin[1], 0);
  ledcWrite(motor_pin[2], 0);
  ledcWrite(motor_pin[3], 0);

  btStop();

  const esp_timer_create_args_t timer_args = {
    .callback = &control_callback,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "control_loop"
  };

  esp_timer_create(&timer_args, &control_timer);

  esp_timer_start_periodic(control_timer, 1000);   // 1000us
  prev_cycle = ESP.getCycleCount();

  rx_data.update_raw_data();
  rx_data.convert_data();
  memcpy(receiver, rx_data.get_data(), sizeof(uint16_t) * 10);
  throttle = (receiver[2] - 1000) / 1000.0f;
  throttle = constrain(throttle, 0.0f, 1.0f);
}

void loop() {
  if (loop_flag) {
    loop_flag = false;
    loop_count++;
    uint32_t now = ESP.getCycleCount();
    uint32_t delta = now - prev_cycle;
    prev_cycle = now;

    dt_loop = delta / 160000000.0f;
    // baro_data.update_data_val();
    // Serial.print("Altitude is: ");
    // Serial.println(baro_data.get_altitude());
    imu_data.update_data_val();
    imu_data.get_accel(&ax, &ay, &az);
    imu_data.get_gyro(&gx, &gy, &gz);

    // Serial.print("ax: ");
    // Serial.print(ax);
    // Serial.print("\tay: ");
    // Serial.print(ay);
    // Serial.print("\taz: ");
    // Serial.print(az);
    // Serial.print("\tgx: ");
    // Serial.print(gx);
    // Serial.print("\tgy: ");
    // Serial.print(gy);
    // Serial.print("\tgz: ");
    // Serial.print(gz);
    // Serial.println();

    float alpha = (2.0f * PI * fc * dt_loop) / (1.0f + 2.0f * PI * fc * dt_loop);

    ax_f += alpha * (ax - ax_f);
    ay_f += alpha * (ay - ay_f);
    az_f += alpha * (az - az_f);

    mahonyUpdate(gx, gy, gz, ax_f, ay_f, az_f, dt_loop);
    angles = getEulerAngles();
    // Serial.print("Roll = ");
    // Serial.print(angles.roll);
    // Serial.print("\tPitch = ");
    // Serial.println(angles.pitch);
    // Serial.print("\tYaw = ");
    // Serial.println(angles.yaw);

    if (arm_flag) {
      if (loop_count % 2 == 0) {
        PID_angle(receiver[0], receiver[1], receiver[3], angles.roll, -angles.pitch, angles.yaw);
      }

      PID_rate(target_roll_rate, target_pitch_rate, target_yaw_rate, gx, gy, -gz, dt_loop);

      float roll_cmd  = PID_roll_rate  / MAX_ROLL_RATE;
      float pitch_cmd = PID_pitch_rate / MAX_PITCH_RATE;
      float yaw_cmd   = PID_yaw_rate   / MAX_YAW_RATE;

      roll_cmd  = constrain(roll_cmd , -1.0f, 1.0f);
      pitch_cmd = constrain(pitch_cmd, -1.0f, 1.0f);
      yaw_cmd   = constrain(yaw_cmd  , -1.0f, 1.0f);

      mixing_pwm(throttle, roll_cmd, pitch_cmd, yaw_cmd, pwm_vals);
    } else {
      for (uint8_t i = 0; i < 4; ++i) {
        pwm_vals[i] = 0;
      }
    }

    for (uint8_t i = 0; i < 4; i++) {
      Serial.print(pwm_vals[i]);
      Serial.print("\t");
    }
    Serial.println();

    if (!(loop_count % 10)) {
      rx_data.update_raw_data();
      rx_data.convert_data();
      memcpy(receiver, rx_data.get_data(), sizeof(uint16_t) * 10);
      // Serial.println(rx_data.is_lost_frame());
      // for (uint8_t i = 0; i < 10; ++i) {
      //   Serial.print(receiver[i]);
      //   Serial.print("\t");
      // }
      // Serial.println();
      throttle = (receiver[2] - 1000) / 1000.0f;
      throttle = constrain(throttle, 0.0f, 1.0f);
      if (receiver[5] > 1500) {
        arm_flag = 1;
      } else {
        arm_flag = 0;
      }
      // float roll = (receiver[0] - 1500) / 500.0f;
      // float pitch = (receiver[1] - 1500) / 500.0f;
      // float yaw = (receiver[3] - 1500) / 500.0f;

      // for (uint8_t i = 0; i < 10; i++) {
      //   Serial.print(receiver[i]);
      //   Serial.print("\t");
      // }
      // Serial.print("Throttle: ");
      // Serial.print(throttle);
      // Serial.print("\tRoll: ");
      // Serial.print(roll);
      // Serial.print("\tPitch: ");
      // Serial.print(pitch);
      // Serial.print("\tYaw: ");
      // Serial.print(yaw);
      // Serial.println();
    }
  }
}
