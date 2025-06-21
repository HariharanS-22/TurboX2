#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

#define PPM_PIN         15
#define MOTOR_LEFT_PIN  18
#define MOTOR_RIGHT_PIN 19
#define SERVO_LEFT      16
#define SERVO_RIGHT     17

// Servo & ESC objects
Servo motorLeft, motorRight, servoL, servoR;

// MPU and timing
MPU6050 mpu;
unsigned long lastLoopTime = 0;
float pitch = 0;  // Filtered pitch
float gyroPitch = 0; // Gyro-only pitch

// Complementary filter
float alpha = 0.96;

// PID variables
float kp = 1.8, ki = 0.0, kd = 0.15;
float pidError = 0, prevError = 0, integral = 0;
float setPoint = 0;

// RC input (PPM values for 10 channels)
volatile uint16_t ppmChannels[10];
volatile uint8_t ppmIndex = 0;
uint32_t lastRise = 0;

void IRAM_ATTR ppmISR() {
  uint32_t now = micros();
  uint16_t duration = now - lastRise;
  lastRise = now;

  if (duration >= 300 && duration <= 2100) {
    if (ppmIndex < 10) {
      ppmChannels[ppmIndex++] = duration;
    }
  } else if (duration >= 3000) {
    ppmIndex = 0; // Start of new frame
  }
}

void setupPPM() {
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);
}

void setupMotors() {
  motorLeft.attach(MOTOR_LEFT_PIN, 1000, 2000);
  motorRight.attach(MOTOR_RIGHT_PIN, 1000, 2000);
  servoL.attach(SERVO_LEFT, 1000, 2000);
  servoR.attach(SERVO_RIGHT, 1000, 2000);
  servoL.write(90);
  servoR.write(90);
}

void setupMPU() {
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
}

void setup() {
  Serial.begin(115200);
  setupMPU();
  setupPPM();
  setupMotors();
  delay(1000);
  lastLoopTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastLoopTime) / 1000.0;
  lastLoopTime = now;

  // Read MPU6050 raw data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert gyro reading to deg/s (MPU6050 default sensitivity = 131 LSB/(deg/s))
  float gyroRate = gy / 131.0;

  // Angle from accelerometer
  float accPitch = atan2(ax, az) * 180.0 / PI;

  // Integrate gyro rate to get angle
  gyroPitch += gyroRate * dt;

  // Complementary filter to combine both
  pitch = alpha * (pitch + gyroRate * dt) + (1 - alpha) * accPitch;

  // RC control: channel 2 = throttle, channel 3 = pitch
  int throttle = map(ppmChannels[2], 1000, 2000, 0, 100);
  int desiredPitch = map(ppmChannels[1], 1000, 2000, -60, 60);

  // PID calculation
  pidError = desiredPitch - pitch;
  integral += pidError * dt;
  float derivative = (pidError - prevError) / dt;
  float output = kp * pidError + ki * integral + kd * derivative;
  prevError = pidError;

  // Mixing
  int motorL = constrain(throttle , 0, 100);
  int motorR = constrain(throttle , 0, 100);
  int servoAngle = constrain(90 + output , 0, 180); // Adjust scaling factor as needed

  // Output to motors
  motorLeft.write(map(motorL, 0, 100, 1000, 2000));
  motorRight.write(map(motorR, 0, 100, 1000, 2000));
  servoL.write(servoAngle);
  servoR.write(180 - servoAngle);

  delay(10); // ~100 Hz loop
}
