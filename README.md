# 🚀 Turbo X2 – Hybrid Bicopter with Self-Balancing Bot
<p align="center">
  <img src="https://drive.google.com/uc?export=view&id=10umfN6Dcz_4xiwXa5b4UARDC6E-SHyCr" alt="Turbo X2" width="600"/>
</p>

**Turbo X2** is a dual-mode robotic system, combining the vertical flight capabilities of a bicopter with the stability and control of a self-balancing bot. This hybrid robot can switch seamlessly between aerial and ground modes using an RC transmitter (Flysky i6) , making it versatile for real-world terrain and tactical operations.

## 📸 Project Overview

<table>
  <tr>
    <td>

- **Modes**: Aerial (Bicopter) and Ground (Balancing Bot)  
- **Control**: ESP32-based real-time PID control with IMU feedback  
- **Switching**: Remote-controlled via PPM input (FlySky i6)  
- **Simulation**: Self-balancing dynamics simulated using Simulink  
- **Design**: Hybrid frame modeled in SolidWorks  

</td>
    <td align = "centre">
      <img src="https://drive.google.com/uc?export=view&id=18cSPEtTr1spDRskpe7otpmmJbWAVOX06" alt="SolidWorks-3D_Model" width="200"/>
    </td>
  </tr>
</table>

---

## 🔧 Hardware Used

<img src="https://drive.google.com/uc?export=view&id=1pqOx0gAKF7ZjbxHlEJdzCmSw-QjJL4KW" alt="Hardware Setup" width="300" align="right" style="margin-left: 20px;"/>

| Component                 | Purpose                          |
|--------------------------|----------------------------------|
| ESP32 WROOM              | Main controller                  |
| MPU6050                  | IMU for angle and acceleration   |
| 920 kV BLDC Motors       | Aerial thrust                    |
| MG995 Servos             | Thrust vector control            |
| 30A ESCs                 | Motor speed control              |
| 200 RPM Johnson Motors   | Ground motion                    |
| L298N Motor Driver       | Drive controller for bot         |
| 11.7V LiPo Battery       | Power source                     |
| FlySky i6 Transmitter    | RC signal input via PPM          |

---

## 🧠 Features

-  **Seamless Mode Switching**  
  Toggle between bicopter and balancing bot using Channel 6 (PPM-based).

-  **Dual PID Control Systems**  
  - Balancing Bot: Real-time upright stability with MPU6050  
  - Bicopter: Thrust stabilization and yaw control

-  **Serial PID Tuning**  
  Adjust PID constants in real-time via Serial Monitor without code re-upload.

-  **Simulation + CAD Integration**  
  Simulink for behavior testing, SolidWorks for mechanical design validation.

---

## 🧪 Simulations & Design

- 📐 **Simulink**  
  Inverted pendulum model used to simulate balancing bot dynamics.

- 🛠️ **SolidWorks**  
  Custom hybrid chassis designed to hold both flight and ground components efficiently.

---

## 🔌 Mode Switching Logic

```cpp
if (ch[5] > 1500) {
  modeBicopter = true; // Aerial mode
} else {
  modeBicopter = false; // Ground mode
}
