#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

MPU6050 mpu;

// === PPM Receiver ===
#define PPM_PIN 15
#define CHANNELS 10
volatile uint16_t ppmVals[CHANNELS];
volatile uint8_t ppmIndex = 0;
volatile uint32_t lastRise = 0;

void IRAM_ATTR ppmISR();  // ISR declaration before use

// === Motor & Servo Pins ===
#define MOTOR_L_PIN 16
#define MOTOR_R_PIN 17
#define SERVO_L_PIN 18
#define SERVO_R_PIN 19

#define ENA 25
#define IN1 26
#define IN2 27
#define ENB 14
#define IN3 12
#define IN4 13

Servo motorL, motorR, servoL, servoR;

// === PID Variables ===
float setPtBot = 0, kPBot = 25, kIBot = 1, kDBot = 0.1;
float setPtB = 0, kPB = 1.5, kIB = 0, kDB = 0.05;
float errBot, intBot, prevErrBot, errB, intB, prevErrB;
unsigned long lastBot, lastB;

// === IMU & Angle ===
float pitch = 0, alpha = 0.96;

// === Setup ===
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 failed!");
    while (1);
  }

  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

  motorL.attach(MOTOR_L_PIN); motorR.attach(MOTOR_R_PIN);
  servoL.attach(SERVO_L_PIN); servoR.attach(SERVO_R_PIN);
  servoL.write(90); servoR.write(90);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  lastBot = lastB = millis();
}

// === Main Loop ===
void loop() {
  handleSerialTuning();
  updateAngle();

  uint16_t ch[CHANNELS];
  noInterrupts();
  memcpy(ch, (const void*)ppmVals, sizeof(ch));  // FIX: cast volatile
  interrupts();

  bool modeBicopter = ch[5] > 1500;  // Switch mode via Channel 6

  if (modeBicopter) {
    float pitchCmd = (ch[1] - 1500) / 10.0;
    int throttle = constrain(ch[2], 1000, 2000);
    float pid = computePid(pitchCmd - pitch, kPB, kIB, kDB, prevErrB, intB, lastB);
    runBicopter(throttle, pid);
  } else {
    float pid = computePid(setPtBot - pitch, kPBot, kIBot, kDBot, prevErrBot, intBot, lastBot);
    runBalancer(pid);
  }
}

// === PPM ISR ===
void IRAM_ATTR ppmISR() {
  uint32_t now = micros(), dur = now - lastRise;
  lastRise = now;
  if (dur > 3000) ppmIndex = 0;
  else if (ppmIndex < CHANNELS) ppmVals[ppmIndex++] = dur;
}

// === Angle Update (Complementary Filter) ===
void updateAngle() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float accPitch = atan2(ax, az) * 180.0 / PI;
  float gyroRate = gy / 131.0;
  float dt = (millis() - lastBot) / 1000.0;
  if (dt <= 0 || dt > 0.5) dt = 0.01;
  pitch = alpha * (pitch + gyroRate * dt) + (1 - alpha) * accPitch;
}

// === Generic PID ===
float computePid(float error, float kp, float ki, float kd, float &prevErr, float &integral, unsigned long &lastT) {
  unsigned long now = millis();
  float dt = (now - lastT) / 1000.0;
  if (dt <= 0 || dt > 0.5) dt = 0.01;
  integral += error * dt;
  float derivative = (error - prevErr) / dt;
  prevErr = error;
  lastT = now;
  return kp * error + ki * integral + kd * derivative;
}

// === Bicopter Control ===
void runBicopter(int throttle, float pid) {
  int mL = constrain(throttle + pid, 1000, 2000);
  int mR = constrain(throttle + pid, 1000, 2000);
  motorL.writeMicroseconds(mL);
  motorR.writeMicroseconds(mR);
  int tilt = constrain(map(pid, -60, 60, 60, 120), 60, 120);
  servoL.write(tilt); servoR.write(tilt);
}

// === Self-Balancing Bot Control ===
void runBalancer(float pidOut) {
  int pwm = constrain(abs(pidOut), 0, 255);
  if (pidOut > 0) driveForward(pwm);
  else driveBackward(pwm);
}

void driveForward(int pwm) {
  analogWrite(ENA, pwm); digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(ENB, pwm); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void driveBackward(int pwm) {
  analogWrite(ENA, pwm); digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  analogWrite(ENB, pwm); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

// === Serial PID Tuning ===
void handleSerialTuning() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n'); cmd.trim();
  if (cmd.length() == 0) return;

  String target = cmd.substring(0, cmd.indexOf(' '));
  cmd = cmd.substring(cmd.indexOf(' ') + 1);
  String param = cmd.substring(0, cmd.indexOf(' '));
  float val = cmd.substring(cmd.indexOf(' ') + 1).toFloat();

  if (target == "BOT") {
    if (param == "KP") kPBot = val;
    if (param == "KI") kIBot = val;
    if (param == "KD") kDBot = val;
    Serial.printf("[BOT PID] KP=%.2f, KI=%.2f, KD=%.2f\n", kPBot, kIBot, kDBot);
  } else if (target == "BICOPTER") {
    if (param == "KP") kPB = val;
    if (param == "KI") kIB = val;
    if (param == "KD") kDB = val;
    Serial.printf("[BICOPTER PID] KP=%.2f, KI=%.2f, KD=%.2f\n", kPB, kIB, kDB);
  }
}
