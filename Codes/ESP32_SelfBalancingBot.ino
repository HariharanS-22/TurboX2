#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Motor A
#define ENA 25
#define IN1 26
#define IN2 27

// Motor B
#define ENB 14
#define IN3 12
#define IN4 13

// PID Variables
float setPoint = 0;  // Desired angle (upright)
float input, output;
float Kp = 25, Ki = 1, Kd = 0.1; // PID tuning values
float previousError = 0, integral = 0;
unsigned long lastTime;

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22); // SDA = 21, SCL = 22 (ESP32 I2C pins)
    mpu.initialize();

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Check MPU6050 connection
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Calculate angle from accelerometer
    input = atan2(ax, az) * 180 / PI;
    
    // Compute PID
    unsigned long now = millis();
    float elapsedTime = (now - lastTime) / 1000.0;
    float error = setPoint - input;
    integral += error * elapsedTime;
    float derivative = (error - previousError) / elapsedTime;
    output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    previousError = error;
    lastTime = now;
    
    // Adjust motor speed
    int speed = constrain(abs(output), 0, 1023); // ESP8266 supports 0-1023 for PWM
    if (output > 0) {
        moveForward(speed);
    } else {
        moveBackward(speed);
    }
}

void moveForward(int speed) {
    analogWrite(ENA, speed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    
    analogWrite(ENB, speed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void moveBackward(int speed) {
    analogWrite(ENA, speed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    
    analogWrite(ENB, speed);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}