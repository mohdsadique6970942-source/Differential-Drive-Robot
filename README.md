# Differential-Drive-Robot
Differential drive mobile robot using ESP32 Dev Kit with IMU 6500, ToF, and encoder motors for precise localization, obstacle detection, and motion control. Implements sensor fusion and PID/non-PID control for accurate path tracking, with ongoing work on mathematical modeling of the robot.

---

## 🧰 Hardware Components
- ESP32  
- IMU Sensor (MPU6050 / BNO055)  
- ToF Sensor (VL53L0X / similar)  
- N20 DC Motors with Encoders  
- Motor Driver (L298N / TB6612FNG)  
- Battery Pack  
- Chassis + Wheels

---

## 🔌 Pin Connections

### 🔹 Motor Driver (TB6612 / L298N Equivalent)

| Function | ESP32 Pin |
|----------|----------|
| PWMA (Left PWM) | 26 |
| AIN1 | 14 |
| AIN2 | 27 |
| PWMB (Right PWM) | 32 |
| BIN1 | 33 |
| BIN2 | 12 |
| STBY | 25 |

---

### 🔹 Encoder Connections

| Encoder Signal | ESP32 Pin |
|----------------|----------|
| Left Encoder A | 34 |
| Left Encoder B | 35 |
| Right Encoder A | 39 |
| Right Encoder B | 36 |

---

### 🔹 Line Sensors (5 Array)

| Sensor | ESP32 Pin |
|--------|----------|
| LS1 | 4 |
| LS2 | 17 |
| LS3 | 16 |
| LS4 | 15 |
| LS5 | 13 |

---

### 🔹 IMU (MPU6050 - I2C)

| Pin | ESP32 |
|-----|-------|
| VCC | 3.3V |
| GND | GND |
| SDA | 21 |
| SCL | 22 |

📌 Install Library:
- `Madgwick` (from Arduino Library Manager)

---

### 🔹 ToF Sensor (VL53L0X - I2C)

| Pin | ESP32 |
|-----|-------|
| VCC | 3.3V |
| GND | GND |
| SDA | 21 |
| SCL | 22 |

📌 Install Library:
- `Adafruit_VL53L0X`

---

### 🔹 Serial Communication

| Function | Value |
|----------|-------|
| Baud Rate | 115200 |
| Interface | USB |

---

## 📁 Code Availability

- Individual sensor codes are available  
- Combined full system code is available
- All paths code are available 
- All codes are inside the `codes` folder  
- Path execution:
  - Straight line (PID + Non-PID with jerk control)  
  - Square path (PID + Non-PID)  
  - Circular path (PID + Non-PID)
 
  ---

## 🎯 Applications
- Autonomous mobile robots  
- SLAM & localization research  
- Robotics competitions  
- Smart navigation systems  

---

## 📚 Learning Outcomes
- Sensor fusion (IMU + ToF + Encoder)  
- Embedded control systems  
- PID tuning and motion control  
- Real-time robotics programming
- Mathematical Modelling

---

## 🚧 Project Status
🔄 Ongoing – Mathematical modelling & advanced improvements in localization and control are under development.

---

## 📄 License
Open-source for educational and research purposes.
