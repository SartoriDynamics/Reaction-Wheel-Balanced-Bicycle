/*
  REWBI - Velocity-control experiment
  Distributed under the MIT License.
  Copyright (c) 2024 by GTI (UFMG).

  Arduino-ESP32 2.0.17 / ESP32Encoder 5.0.0
  Serial columns: velocity (m/s), traction command (V), time from step (s).
*/

#include <Wire.h>
#include <ESP32Encoder.h>

#define INTERNAL_LED 2
#define MPU 0x68

#define SERVO_PWM 32
#define STEERING_CENTER 78
#define SERVO_TIMER_BIT 16
#define SERVO_BASE_FREQ 50
#define SERVO_PWM_CH 3

#define NIDEC_TIMER_BIT 8
#define NIDEC_BASE_FREQ 20000

#define BRAKE1 4
#define NIDEC1_PWM 16
#define DIR1 17
#define ENCA_1 18
#define ENCB_1 19
#define NIDEC1_PWM_CH 1

#define BRAKE2 14
#define NIDEC2_PWM 27
#define DIR2 26
#define ENCA_2 25
#define ENCB_2 33
#define NIDEC2_PWM_CH 0

const float Ts = 0.01f;
const float Tv = 0.04f;
const unsigned long BALANCE_PERIOD_MS = 10;
const unsigned long VELOCITY_PERIOD_MS = 40;

const unsigned long RECORD_START_MS = 60000;
const unsigned long REFERENCE_STEP_MS = 61000;
const unsigned long PRINT_START_MS = 120000;
const unsigned long PRINT_PERIOD_MS = 20000;

const float SPEED_REFERENCE = 0.5f;
const float VOLTAGE_LIMIT = 12.0f;
const float VOLTAGE_TO_PWM = 21.3f;
const float COUNTS_PER_RADIAN = 63.7f;
const float TRACTION_COUNTS_PER_MM = 6.12f;

// 1000 samples at 40 ms preserve the original 40-second recording window.
const int DADOSMAX = 1000;
float dados[DADOSMAX][3];
int count = 0;

ESP32Encoder NIDEC1_ENC;
ESP32Encoder NIDEC2_ENC;
int64_t traction_count_past = 0;

float Q_angle = 0.001f;
float Q_bias = 0.005f;
float R_meas = 1.0f;
float roll_angle = 0.0f;
float bias = 0.0f;
float P[2][2] = {{1, 0}, {0, 1}};
float K[2] = {0, 0};

// Balance gains from LQR_experiment.ino.
float K1 = -188.0f;
float K2 = -19.8f;
float K3 = -0.112f;
float K4 = 0.011f;
float theta = 0.0f;
float theta_dot = 0.0f;
float omega = 0.0f;
float integral = 0.0f;
float integral_past = 0.0f;
float U = 0.0f;

// Velocity PI from the path-planning code, with the article's reference.
float kp = 10.0f;
float ki = 30.0f;
float bike_speed = 0.0f;  // mm/s
float vel_ref = 0.0f;     // m/s
float erro_integral = 0.0f;
float U2 = 0.0f;

unsigned long previousT = 0;
unsigned long previousTv = 0;
unsigned long previousTc = 0;
unsigned long experiment_start = 0;

bool balance_active = false;
bool data_ready = false;
bool export_enabled = false;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(INTERNAL_LED, OUTPUT);
  digitalWrite(INTERNAL_LED, HIGH);

  NIDECsetup();
  IMUsetup();
  SERVOsetup();

  delay(1500);
  for (int i = 0; i < 400; i++) {
    IMUread();
    delay(5);
  }

  resetControllers();
  digitalWrite(INTERNAL_LED, LOW);
}

void loop() {
  equilibra();
  velControl();
  printa();
}

void equilibra() {
  if (export_enabled) {
    return;
  }

  unsigned long currentT = millis();
  if (currentT - previousT < BALANCE_PERIOD_MS) {
    return;
  }
  previousT = currentT;

  IMUread();

  if (abs(roll_angle) < 20) {
    balance_active = true;
    digitalWrite(BRAKE1, HIGH);
    digitalWrite(BRAKE2, data_ready ? LOW : HIGH);

    theta += theta_dot * Ts;
    integral = -NIDEC1_ENC.getCount() / COUNTS_PER_RADIAN;
    omega = -(integral - integral_past) / Ts;
    integral_past = integral;

    U = -(K1 * theta + K2 * theta_dot + K3 * omega + K4 * integral);
    U = constrain(U, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
    MOTOR1cmd((int)(U * VOLTAGE_TO_PWM));

    // Preserve the 10-ms speed measurement used in the path-planning code.
    int64_t traction_count = NIDEC2_ENC.getCount();
    int64_t count_increment = traction_count - traction_count_past;
    traction_count_past = traction_count;
    bike_speed = count_increment / (Ts * TRACTION_COUNTS_PER_MM);
    return;
  }

  balance_active = false;
  stopMotors();
  SERVOangle(STEERING_CENTER);
  digitalWrite(INTERNAL_LED, HIGH);
  vel_ref = 0.0f;
  erro_integral = 0.0f;
  U2 = 0.0f;

  // Freeze complete or interrupted recordings for USB export.
  if (count > 0) {
    data_ready = true;
    export_enabled = true;
    return;
  }

  delay(5000);
  for (int i = 0; i < 400; i++) {
    IMUread();
    delay(5);
  }

  resetControllers();
  digitalWrite(INTERNAL_LED, LOW);
}

void velControl() {
  if (!balance_active || data_ready || export_enabled) {
    return;
  }

  unsigned long currentTv = millis();
  if (currentTv - previousTv < VELOCITY_PERIOD_MS) {
    return;
  }
  previousTv = currentTv;

  unsigned long elapsed = currentTv - experiment_start;
  vel_ref = elapsed >= REFERENCE_STEP_MS ? SPEED_REFERENCE : 0.0f;

  float measured_speed = bike_speed / 1000.0f;
  float erro = vel_ref - measured_speed;
  erro_integral += erro * Tv;

  U2 = kp * erro + ki * erro_integral;
  U2 = constrain(U2, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  MOTOR2cmd((int)(U2 * VOLTAGE_TO_PWM));

  if (elapsed >= RECORD_START_MS && count < DADOSMAX) {
    dados[count][0] = measured_speed;
    dados[count][1] = U2;
    dados[count][2] = ((float)elapsed - REFERENCE_STEP_MS) / 1000.0f;
    count++;
  }

  if (count == DADOSMAX) {
    data_ready = true;
    vel_ref = 0.0f;
    erro_integral = 0.0f;
    U2 = 0.0f;
    MOTOR2cmd(0);
    digitalWrite(BRAKE2, LOW);
  }
}

void printa() {
  if (!export_enabled || !data_ready || count == 0) {
    return;
  }

  unsigned long currentTc = millis();
  if (currentTc - experiment_start < PRINT_START_MS ||
      currentTc - previousTc < PRINT_PERIOD_MS) {
    return;
  }
  previousTc = currentTc;

  // Print one complete recording. Only populated rows are exported.
  for (int i = 0; i < count; i++) {
    Serial.print(dados[i][0], 5);
    Serial.print(' ');
    Serial.print(dados[i][1], 5);
    Serial.print(' ');
    Serial.println(dados[i][2], 5);
  }
  Serial.println();
}

void resetControllers() {
  theta = 0.0f;
  theta_dot = 0.0f;
  omega = 0.0f;
  integral = 0.0f;
  integral_past = 0.0f;
  U = 0.0f;
  bike_speed = 0.0f;
  vel_ref = 0.0f;
  erro_integral = 0.0f;
  U2 = 0.0f;

  NIDEC1_ENC.clearCount();
  NIDEC2_ENC.clearCount();
  traction_count_past = 0;

  experiment_start = millis();
  previousT = experiment_start;
  previousTv = experiment_start;
  previousTc = 0;
}

void stopMotors() {
  MOTOR1cmd(0);
  MOTOR2cmd(0);
  digitalWrite(BRAKE1, LOW);
  digitalWrite(BRAKE2, LOW);
}

void IMUsetup() {
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(1 << 3);
  Wire.endTransmission();

  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission();
}

void SERVOsetup() {
  pinMode(SERVO_PWM, OUTPUT);
  ledcSetup(SERVO_PWM_CH, SERVO_BASE_FREQ, SERVO_TIMER_BIT);
  ledcAttachPin(SERVO_PWM, SERVO_PWM_CH);
  SERVOangle(STEERING_CENTER);
}

void NIDECsetup() {
  pinMode(BRAKE1, OUTPUT);
  pinMode(BRAKE2, OUTPUT);
  digitalWrite(BRAKE1, LOW);
  digitalWrite(BRAKE2, LOW);

  pinMode(DIR1, OUTPUT);
  ledcSetup(NIDEC1_PWM_CH, NIDEC_BASE_FREQ, NIDEC_TIMER_BIT);
  ledcAttachPin(NIDEC1_PWM, NIDEC1_PWM_CH);
  MOTOR1cmd(0);

  pinMode(DIR2, OUTPUT);
  ledcSetup(NIDEC2_PWM_CH, NIDEC_BASE_FREQ, NIDEC_TIMER_BIT);
  ledcAttachPin(NIDEC2_PWM, NIDEC2_PWM_CH);
  MOTOR2cmd(0);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  NIDEC1_ENC.attachFullQuad(ENCB_1, ENCA_1);
  NIDEC2_ENC.attachFullQuad(ENCB_2, ENCA_2);
  NIDEC1_ENC.clearCount();
  NIDEC2_ENC.clearCount();
}

void IMUread() {
  int16_t ax, ay, az, gx;

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read();
  Wire.read();  // Temperature is not used.
  gx = Wire.read() << 8 | Wire.read();
  Wire.read();
  Wire.read();
  Wire.read();
  Wire.read();  // The other gyroscope axes are not used.

  float ax_angle = atan2(ay, sqrt(ax * ax + az * az)) * 57.3;
  gx = gx / 65.5;

  roll_angle += (gx - bias) * Ts;

  P[0][0] += (Q_angle - P[0][1] - P[1][0]) * Ts;
  P[0][1] += -P[1][1] * Ts;
  P[1][0] += -P[1][1] * Ts;
  P[1][1] += Q_bias * Ts;

  K[0] = P[0][0] / (P[0][0] + R_meas);
  K[1] = P[1][0] / (P[0][0] + R_meas);

  roll_angle += K[0] * (ax_angle - roll_angle);
  bias += K[1] * (ax_angle - roll_angle);

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  theta_dot = (gx - bias) / 57.3;
}

void MOTOR1cmd(int sp) {
  if (sp < 0) {
    digitalWrite(DIR1, HIGH);
    sp = -sp;
  } else {
    digitalWrite(DIR1, LOW);
  }
  ledcWrite(NIDEC1_PWM_CH, sp > 255 ? 0 : 255 - sp);
}

void MOTOR2cmd(int sp) {
  if (sp < 0) {
    digitalWrite(DIR2, HIGH);
    sp = -sp;
  } else {
    digitalWrite(DIR2, LOW);
  }
  ledcWrite(NIDEC2_PWM_CH, sp > 255 ? 0 : 255 - sp);
}

void SERVOangle(int angle) {
  float dutyCycle = map(angle, 0, 180, 1500, 7900);
  ledcWrite(SERVO_PWM_CH, (int)dutyCycle);
}