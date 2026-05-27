# Self-Balancing Robot

An Arduino-based two-wheeled self-balancing robot. Uses an MPU6050 IMU fused through a Kalman filter and a PID controller to maintain upright balance, with Bluetooth remote control.

https://github.com/user-attachments/assets/balanceDemo.mp4

---

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | Arduino Uno (or compatible) |
| IMU | MPU6050 (I²C, address `0x68`) |
| Motor driver | L298N dual H-bridge |
| Motors | 2× DC gear motors |
| Bluetooth | HC-05 module (SoftwareSerial) |
| Power | 7–12 V battery pack |

---

## Wiring

### MPU6050 → Arduino
| MPU6050 | Arduino |
|---------|---------|
| VCC | 5 V |
| GND | GND |
| SDA | A4 |
| SCL | A5 |

### L298N → Arduino
| L298N | Arduino pin |
|-------|-------------|
| ENA (Motor A speed) | 9 (PWM) |
| IN1 | 4 |
| IN2 | 5 |
| ENB (Motor B speed) | 10 (PWM) |
| IN3 | 6 |
| IN4 | 7 |

### HC-05 Bluetooth → Arduino
| HC-05 | Arduino pin |
|-------|-------------|
| TX | 12 (SoftwareSerial RX) |
| RX | 13 (SoftwareSerial TX) |

---

## Sketches

| Folder | Purpose |
|--------|---------|
| `balancingRobotMK1/` | **Main firmware** — Kalman filter, PID loop, Bluetooth control |
| `BLUETOOTHWORKS/` | Bluetooth connectivity test — toggle an LED over BT |
| `roll/` | IMU test — streams pitch angle to Serial Monitor |

### Flashing

1. Open the desired `.ino` file in Arduino IDE.
2. Select your board and COM port under **Tools**.
3. Click **Upload**.

---

## How It Works

1. The MPU6050 provides raw accelerometer and gyroscope readings at ~3 ms intervals.
2. A **Kalman filter** fuses the two signals to produce a low-noise pitch angle estimate.
3. A **PID controller** computes motor power based on the error between the current angle and the target balance angle (~−1.6°).
4. Motor output is additionally smoothed with exponential smoothing + rate limiting to reduce oscillation.

### PID Tuning (MK1)

| Parameter | Value |
|-----------|-------|
| Kp | 131.19 |
| Ki | 0.01 |
| Kd | 0.07 |
| Balance angle | −1.6° |

---

## Bluetooth Commands

Connect to the HC-05 module (default baud: 9600) and send single-character commands:

| Command | Action |
|---------|--------|
| `A` | Start balancing (required on boot) |
| `F` | Lean forward |
| `B` | Lean backward |
| `0` | Return to balance |
| `P` | Print current PID values and pitch angle |

---

## Dependencies

Install via the Arduino Library Manager:

- [MPU6050 by Electronic Cats](https://github.com/electroniccats/mpu6050) (or jrowberg's I2Cdevlib)
- `Wire.h` — built-in
- `SoftwareSerial.h` — built-in

---

## CAD

`balancingRobot.f3z` — Fusion 360 archive of the robot chassis. Open in Fusion 360 via **File → Open → Open from my computer**.

---

## Media

`balanceDemo.mp4` — demo of the robot balancing.  
`balanceMARK0.mp4` — early prototype demo.