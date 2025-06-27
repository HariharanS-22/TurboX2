#  👾 CODE Overview 
## 📐 IMU + PID Control
```cpp

mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
input = atan2(ax, az) * 180 / PI;  // Calculate tilt angle

// PID logic
error = setPoint - input;
integral += error * elapsedTime;
derivative = (error - previousError) / elapsedTime;
output = (Kp * error) + (Ki * integral) + (Kd * derivative);
```
- The MPU6050 gives the tilt angle.
- The setpoint is ideally 0° (upright).
- The PID loop compares the current angle to the setpoint.
- Based on the result (`output`), the bot moves motors forward or backward to correct the tilt.

## 📡 PPM Input Handling - Bicopter Mode
```cpp
volatile uint16_t ppmChannels[10];  // Stores pulse width for 10 channels
volatile uint8_t ppmIndex = 0;
uint32_t lastRise = 0;

void IRAM_ATTR ppmISR() {
    uint32_t now = micros();
    uint16_t duration = now - lastRise;
    lastRise = now;

    if (duration >= 300 && duration <= 2100) {
        if (ppmIndex < 10)
            ppmChannels[ppmIndex++] = duration;
    } else if (duration >= 3000) {
        ppmIndex = 0;  // Start of new frame
    }
}
```
### PPM (Pulse Position Modulation) 
- Sends multiple RC channel signals  - single wire
- Each pulse gap represents a channel value. 
- Uses hardware interrupt to capture pulses (`attachInterrupt()`)
- `micros()` function for precise pulse timing
- Channels stored sequentially: CH1 = ppmChannels[0], CH2 = ppmChannels[1]
- Handles up to 10 channels in a single frame

## 🔀 Mode Switching
```cpp
bool modeBicopter = ch[5] > 1500;

if (modeBicopter) {
    float pitchCmd = (ch[1] - 1500) / 10.0;
    int throttle = constrain(ch[2], 1000, 2000);
    float pid = computePid(pitchCmd - pitch, kPB, kIB, kDB, prevErrB, intB, lastB);
    runBicopter(throttle, pid);
} else {
    float pid = computePid(setPtBot - pitch, kPBot, kIBot, kDBot, prevErrBot, intBot, lastBot);
    runBalancer(pid);
}
```
- Uses RC Channel 6 (ch[5]) to toggle between modes
   - ch[5] > 1500 → Bicopter Mode
   - ch[5] ≤ 1500 → Balancer Mode

- Uses `noInterrupts()` and `interrupts()` to avoid race conditions
- Mode status stored in `bool modeBicopter`

## ⚙️ Serial PID Tuning
```cpp
void handleSerialTuning() {
    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    String target = cmd.substring(0, cmd.indexOf(' '));
    cmd = cmd.substring(cmd.indexOf(' ') + 1);
    String param = cmd.substring(0, cmd.indexOf(' '));
    float val = cmd.substring(cmd.indexOf(' ') + 1).toFloat();

    if (target == "BOT") {
        if (param == "Kp") kPBot = val;
        else if (param == "Ki") kIBot = val;
        else if (param == "Kd") kDBot = val;
    } else if (target == "BICOPTER") {
        if (param == "Kp") kPB = val;
        else if (param == "Ki") kIB = val;
        else if (param == "Kd") kDB = val;
    }
}
```

- Updates appropriate PID variables
  - `kPBot, kIBot, kDBot` (for Self-balancing Bot)
  - `kPB, kIB, kDB` (for Bicopter)
- Live updates shown via `Serial.print()` for confirmation

### Advantages
- Real-time PID tuning without re-uploading code
- Enables smooth switching and stabilization during field testing
