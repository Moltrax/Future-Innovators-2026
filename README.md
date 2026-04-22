# WRO 2026 Future Engineers – Autonomous Vehicle

![WRO 2026](https://img.shields.io/badge/WRO-2026-blue)
![Platform](https://img.shields.io/badge/Platform-ESP32-green)
![Framework](https://img.shields.io/badge/Framework-Arduino-teal)
![Build](https://img.shields.io/badge/Build-PlatformIO-orange)

An autonomous 4-wheeled vehicle built for the **World Robot Olympiad 2026 Future Engineers** category. The vehicle navigates a randomized rectangular racetrack for 3 laps, avoids colored pillars by passing them on the correct side, and performs parallel parking after completing the obstacle challenge.

---

## Table of Contents

- [Challenge Overview](#challenge-overview)
- [Hardware](#hardware)
- [Software Architecture](#software-architecture)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Building and Flashing](#building-and-flashing)
  - [PixyCam2 Signature Training](#pixycam2-signature-training)
- [Configuration](#configuration)
- [How It Works](#how-it-works)
  - [Dual-Core Design](#dual-core-design)
  - [State Machine](#state-machine)
  - [Wall Centering PD Controller](#wall-centering-pd-controller)
  - [Pillar Avoidance](#pillar-avoidance)
  - [Lap Counting](#lap-counting)
  - [Parallel Parking](#parallel-parking)
- [Tuning Guide](#tuning-guide)
- [Start Procedure](#start-procedure)
- [Track Specifications](#track-specifications)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Challenge Overview

The WRO Future Engineers challenge requires a fully autonomous vehicle to:

1. **Drive 3 laps** in a randomly assigned direction (clockwise or counter-clockwise) on a rectangular track with 4 corners and 4 straights.
2. **Stay between walls** — black inner and outer walls, 100 mm tall, with a track width of 600–1000 mm.
3. **Avoid colored pillars** — red pillars must be passed on the **right**, green pillars on the **left**. Pillars must not be pushed outside their 85 mm evaluation circle.
4. **Parallel park** after 3 laps (obstacle challenge) — the vehicle must fit entirely inside a parking rectangle bounded by two magenta markers, aligned parallel to the outer wall.

Two challenge modes are supported:

| Mode | Description |
|------|-------------|
| **Open Challenge** | No pillars. Complete 3 laps and stop in the starting section. |
| **Obstacle Challenge** | Pillars present. Complete 3 laps, then find and enter the parking lot. |

---

## Hardware

| Component | Model / Type | Interface | Purpose |
|-----------|-------------|-----------|---------|
| **MCU** | ESP32-WROOM-32 | — | Main controller, dual-core |
| **Camera** | PixyCam2 | SPI | Detects red, green, and magenta objects |
| **Ultrasonic Left** | HC-SR04 | GPIO | Distance to left wall |
| **Ultrasonic Right** | HC-SR04 | GPIO | Distance to right wall |
| **Gyroscope** | MPU6050 | I2C | Heading tracking, lap/corner counting |
| **Steering Servo** | Standard servo | PWM | Front axle steering (90 degrees = straight) |
| **Drive Motors** | 2x DC motor | H-Bridge PWM | Rear axle drive (controlled as single unit) |
| **Start Button** | Momentary push button | GPIO (active LOW) | Initiates autonomous run |

### Wiring Summary

ESP32 Pin Component

---

GPIO 12 Ultrasonic Left TRIG
GPIO 14 Ultrasonic Left ECHO
GPIO 27 Ultrasonic Right TRIG
GPIO 26 Ultrasonic Right ECHO
GPIO 32 Motor PWM
GPIO 33 Motor DIR1 (H-Bridge)
GPIO 25 Motor DIR2 (H-Bridge)
GPIO 13 Steering Servo PWM
GPIO 21 MPU6050 SDA
GPIO 22 MPU6050 SCL
GPIO 4 Start Button (INPUT_PULLUP)
SPI bus PixyCam2 (default SPI pins)

---

## Software Architecture

The firmware uses a **dual-core architecture** on the ESP32 to separate time-critical sensor reading from control logic:

            SENSOR LAYER (Core 0, ~40 Hz)
+----------------------------------------------+
|  Ultrasonic L/R  ->  Median Filter (w=5)     |
|  PixyCam2        ->  Pillar + Parking det.   |
|  MPU6050         ->  Heading (Comp. Filter)  |
+---------------------+------------------------+
                      |  SensorData struct + mutex
+---------------------+------------------------+
|       CONTROL LAYER (Core 1, ~100 Hz)        |
|  Finite State Machine (FSM)                  |
|  PD Controller (wall centering)              |
|  Motor Ramp Generator                        |
|  Servo Slew-Rate Limiter                     |
+----------------------------------------------+

**Core 0** runs a FreeRTOS task that reads all sensors at ~40 Hz and writes results to a shared SensorData struct protected by a mutex.

**Core 1** runs the Arduino loop() at ~100 Hz, copies the latest sensor data, runs the state machine, and updates actuators.

This separation ensures that slow ultrasonic pulseIn() calls never block the fast control loop.

---

## Project Structure

project-root/
├── platformio.ini # PlatformIO build configuration
└── src/
├── main.cpp # setup(), loop(), dual-core task, entry point
├── config.h # ALL pin definitions, tuning parameters, constants
├── utils/
│ ├── filters.h # MedianFilter and SchmittTrigger structs
│ └── filters.cpp
├── sensors/
│ ├── ultrasonic.h # HC-SR04 read + median filter
│ ├── ultrasonic.cpp
│ ├── pixy_cam.h # PixyCam2: red/green/magenta detection
│ ├── pixy_cam.cpp
│ ├── gyro.h # MPU6050: heading, lap count, corner count
│ └── gyro.cpp
├── control/
│ ├── pd_controller.h # Reusable PD controller (derivative-kick avoidance)
│ ├── pd_controller.cpp
│ ├── state_machine.h # Full FSM for open + obstacle challenge
│ └── state_machine.cpp
└── actuators/
├── motor.h # H-Bridge: deadzone compensation, ramp generator
├── motor.cpp
├── servo_ctrl.h # Servo: slew-rate limited steering
└── servo_ctrl.cpp

---

### Module Descriptions

| Module | Responsibility |
|--------|---------------|
| config.h | Single source of truth for all pins, tuning constants, and game parameters |
| filters | Reusable MedianFilter (window=5) and SchmittTrigger (hysteresis) structs |
| ultrasonic | HC-SR04 distance measurement with timeout protection and median filtering |
| pixy_cam | PixyCam2 block detection, signature filtering, pinhole distance estimation |
| gyro | MPU6050 yaw integration, complementary filter fusion with US data, lap/corner detection |
| pd_controller | Generic PD controller with derivative computed on measurement (not error) to prevent kick |
| state_machine | 13-state FSM covering driving, pillar avoidance, parking, and both challenge modes |
| motor | H-Bridge control with deadzone compensation and acceleration/deceleration ramping |
| servo_ctrl | Servo output with slew-rate limiting for smooth, predictable steering |

---

## Getting Started

### Prerequisites

- **PlatformIO** (CLI or IDE plugin for VS Code)
- **ESP32 board** (ESP32-WROOM-32 or compatible)
- **PixyCam2** with trained color signatures (see below)
- All hardware wired according to the pin table in config.h

### Building and Flashing

```bash
# Clone the repository
git clone https://github.com/your-team/wro2026-future-engineers.git
cd wro2026-future-engineers

# Build
pio run

# Flash to ESP32
pio run --target upload

# Monitor serial output
pio device monitor --baud 115200
```

---

### PixyCam2 Signature Training

Before running, the PixyCam2 must be trained to recognize three color signatures using the **PixyMon** desktop application:

| Signature | Color | RGB Reference | Object |
|-----------|-------|---------------|--------|
| **1** | Red | (238, 39, 55) | Traffic pillars — pass on RIGHT |
| **2** | Green | (68, 214, 44) | Traffic pillars — pass on LEFT |
| **3** | Magenta | (255, 0, 255) | Parking lot markers |

**Training steps:**

1. Connect PixyCam2 to PC via USB
2. Open PixyMon
3. Place a red pillar in front of the camera then select Action and Set Signature 1
4. Repeat for green pillar as Signature 2
5. Repeat for magenta marker as Signature 3
6. Adjust signature ranges if needed for consistent detection under competition lighting
7. Save and disconnect

---

## Configuration

All tunable parameters are in **src/config.h**. No magic numbers exist in logic code.

### Challenge Setup

```cpp
#define CHALLENGE_MODE      1     // 0 = Open Challenge, 1 = Obstacle Challenge
#define DRIVING_DIRECTION   1     // 1 = Clockwise, -1 = Counter-Clockwise
```

Change these two values to match the round requirements before flashing.


### Key Tuning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| KP_STEER | 1.5 | Proportional gain for wall-centering PD |
| KD_STEER | 0.8 | Derivative gain for wall-centering PD |
| SPEED_MAX | 220 | Straight-line cruise speed (PWM 0-255) |
| SPEED_PARK | 70 | Speed during parking maneuvers |
| EVADE_TRIGGER_CM | 25 | Distance at which hard evasion steering begins |
| REACTION_DIST_FAST | 60 | Look-ahead distance at max speed (cm) |
| COMP_FILTER_ALPHA | 0.95 | Gyro weight in complementary filter (0-1) |
| SERVO_CENTER | 90 | Servo angle for straight-ahead steering |
| MOTOR_DEADZONE | 40 | Minimum PWM to overcome motor static friction |
| MAX_SLEW_RATE | 5.0 | Max servo angle change per control cycle (degrees) |

See config.h for the complete list with documentation.

---

## How It Works

### Dual-Core Design

The ESP32 two cores are dedicated to separate responsibilities:

- **Core 0 — Sensor Task** (sensorTask): Reads ultrasonics (with median filtering), queries the PixyCam2 for blocks, and updates the gyroscope heading. Runs every 25 ms (~40 Hz). Writes results to a shared SensorData struct protected by a FreeRTOS mutex.

- **Core 1 — Control Loop** (loop()): Copies the latest sensor data (under mutex), runs the finite state machine, computes PD steering corrections, and updates motor/servo outputs. Runs every 10 ms (~100 Hz).

This separation ensures that slow ultrasonic pulseIn() calls never block the fast control loop.

### State Machine

The FSM has **13 states** covering all phases of competition:

```
+-------------------+
| WAIT_FOR_START    |<--- Power on, setup complete
+---------+---------+
          | Button pressed
+---------v---------+
|       DRIVE       |<--- Wall centering with PD
+--+------+------+--+
   |      |      |
   |      |      +---- lapCount >= 3 ----> FIND_PARK (obstacle)
   |      |                                or OPEN_FINISH (open)
   |      +---- Pillar detected within reaction distance
   |             |
   |      +------v--------+
   |      |APPROACH_PILLAR|  Slow down, pre-position
   |      +---+-------+---+
   |          |       |
   |     +----v--+ +--v-----+
   |     |EVADE_L| |EVADE_R |  Hard steer past pillar
   |     +---+---+ +---+----+
   |         +-----+----+
   |        +------v------+
   |        |  STRAIGHTEN  |  Return to center
   |        +------+------+
   |               +----------> DRIVE
   |
+--v-----------+    +--------------+
|  FIND_PARK   |--->|PARKING_ALIGN |
+--------------+    +------+-------+
                    +------v-------+
                    |PARKING_ENTER |  Turn ~90 degrees into lot
                    +------+-------+
                    +------v-------+
                    |PARKING_CENTER|  Center + align parallel
                    +------+-------+
                    +------v-------+
                    |  PARK_DONE   |---> DONE
                    +--------------+
```

**Full state list:**

| State | Description |
|-------|-------------|
| WAIT_FOR_START | Idle until start button pressed |
| DRIVE | Center between walls using PD control, drive at SPEED_MAX |
| APPROACH_PILLAR | Pillar detected within reaction distance, reduce speed, pre-position |
| EVADE_LEFT | Hard steer left to pass green pillar on its left |
| EVADE_RIGHT | Hard steer right to pass red pillar on its right |
| STRAIGHTEN | Return to center using PD, resume DRIVE when centered |
| FIND_PARK | After 3 laps: slow down, scan for magenta markers |
| PARKING_ALIGN | Position vehicle alongside parking lot opening |
| PARKING_ENTER | Turn into lot with gyro-assisted ~90 degree turn |
| PARKING_CENTER | Drive into lot, center between markers, align parallel |
| PARK_DONE | Stop motors, transition to DONE |
| OPEN_FINISH | After 3 laps in open challenge: stop in starting section |
| DONE | Final idle state |

### Wall Centering PD Controller

During DRIVE and STRAIGHTEN states, a PD controller keeps the vehicle centered between walls:

```
error      = distLeft - distRight     (0 = centered)
steerAngle = SERVO_CENTER + Kp * error - Kd * d(measured)/dt
```

The derivative is computed on the **measurement** (not the error) to avoid derivative kick when the setpoint changes. There is intentionally no I-term — the vehicle moves fast enough that integral windup would cause oscillation.

### Pillar Avoidance

Pillar avoidance uses a multi-phase approach:

1. **Detection** (DRIVE to APPROACH_PILLAR): PixyCam2 detects a red or green block within the speed-dependent reaction distance. Speed begins decreasing linearly.

2. **Pre-positioning** (APPROACH_PILLAR): The vehicle steers to the opposite side of the pillar to create clearance. Green pillar means steer slightly right. Red pillar means steer slightly left.

3. **Evasion** (EVADE_LEFT / EVADE_RIGHT): When the pillar is within EVADE_TRIGGER_CM, hard steering is applied. The state exits when the pillar leaves the camera frame or a timeout elapses.

4. **Recovery** (STRAIGHTEN): PD controller re-centers the vehicle between walls before resuming DRIVE.

Distance estimation uses a **pinhole camera model**:

```
distance_cm = (REF_DIST_CM * REF_HEIGHT_PX) / measured_block_height
```

Calibrated against the known 100 mm pillar height.

### Lap Counting

Two complementary methods track laps:

1. **Heading accumulation**: The gyroscope tracks total heading change. Every 360 degrees of accumulated rotation (sign-dependent on DRIVING_DIRECTION) increments the lap counter.

2. **Corner counting**: Each approximately 90 degree heading change increments a corner counter. 4 corners = 1 lap. This provides redundancy if heading drift causes the primary counter to misfire.

The heading is maintained by a **complementary filter** that fuses the MPU6050 gyro rate (high-frequency, no drift short-term) with an ultrasonic-derived angle estimate (low-frequency, absolute reference):

```
heading = alpha * (heading + gyroRate * dt) + (1 - alpha) * usAngle
```

Where alpha = 0.95 by default.

### Parallel Parking

After 3 laps in obstacle mode, the vehicle:

1. **FIND_PARK**: Slows down and scans for magenta markers (PixyCam) or a sudden gap in ultrasonic readings (parking lot opening).

2. **PARKING_ALIGN**: Drives alongside the parking lot, using the magenta marker position in the camera frame to align.

3. **PARKING_ENTER**: Executes an approximately 90 degree turn into the parking lot, monitored by the gyroscope.

4. **PARKING_CENTER**: Uses PD control on ultrasonic readings to center between the lot boundaries and align parallel to the outer wall. Completion criteria: |distL - distR| < 2 cm.

---

## Tuning Guide

### Step 1: Verify Sensors

Flash the firmware and monitor serial output in WAIT_FOR_START state. The sensor task runs continuously. Verify:

```
[WAIT_FOR_START] L=25 R=24 H=0.0 Lap=0 Spd=0 Str=90
```

- L and R should reflect actual wall distances in cm
- H should be stable near 0 when stationary
- Place a pillar in front of the camera and verify P:s1 d=XX or P:s2 d=XX appears

### Step 2: Motor Deadzone

Place the vehicle on blocks (wheels off ground). Gradually increase MOTOR_DEADZONE in config.h until the minimum nonzero PWM reliably spins the wheels. Typical range: 30-60.

### Step 3: PD Gains

1. Set KD_STEER = 0, start with KP_STEER = 1.0
2. Run on the track — vehicle should oscillate between walls
3. Increase KD_STEER until oscillation is damped (start with 0.5)
4. If response is too sluggish, increase KP_STEER
5. If vehicle oscillates or overshoots, increase KD_STEER or decrease KP_STEER

### Step 4: Pillar Distances

Adjust EVADE_TRIGGER_CM and REACTION_DIST values based on actual stopping/turning performance at your vehicle speed. Faster vehicles need larger values.

### Step 5: Parking

Parking is the most sensitive routine. Tune on the actual parking lot:

1. Verify magenta detection range and accuracy
2. Adjust PARKING_TURN_DEG (should be slightly less than 90 to account for momentum)
3. Tune KP_PARK / KD_PARK for smooth centering at low speed
4. Adjust PARKING_PARALLEL_TOL — tighter tolerance means more precise but may never settle

---

## Start Procedure

The start procedure follows WRO regulations exactly:

```
1. Place vehicle in starting zone
2. Power OFF
3. Flip the single hardware power switch to ON
4. ESP32 boots, setup() runs:
   - Serial initialized
   - I2C + MPU6050 initialized
   - Gyro offset calibration (~3 seconds, vehicle must be stationary)
   - PixyCam2 initialized
   - Servo centered
   - Motor initialized (PWM = 0)
   - Start button configured (INPUT_PULLUP)
   - Sensor task created on Core 0
   - FSM enters WAIT_FOR_START
5. Vehicle is idle — no movement, all sensors running
6. Press the single start button
7. Vehicle begins autonomous operation
```

No data entry, no manual calibration, and no physical adjustments occur between power-on and button press. The only human interactions are the power switch (step 3) and the start button (step 6).

---

## Track Specifications

```
+---------------------------------------------------+
|                                                   |
|   +-----------------------------------------+     |
|   |                                         |     |
|   |              Inner Area                 |     |
|   |           (3000 x 3000 mm)              |     |
|   |                                         |     |
|   +-----------------------------------------+     |
|                                                   |
|              Track Width: 1000 mm                 |
|         (600-1000 mm in open challenge)           |
|                                                   |
+---------------------------------------------------+
```

| Property | Value |
|----------|-------|
| Wall height | 100 mm (black) |
| Surface | White |
| Corner markers | Orange and Blue lines (20 mm thick) |
| Pillar base | 50 x 50 mm, 100 mm tall |
| Pillar eval circle | 85 mm diameter |
| Parking lot width | 200 mm |
| Parking lot length | 1.5 x vehicle length |
| Parking markers | Magenta blocks (200 x 20 x 100 mm) |
| Sections per lap | 8 (4 straights + 4 corners) |

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Vehicle does not start | Start button wiring or pull-up | Check GPIO 4, verify INPUT_PULLUP, test with serial monitor |
| Erratic steering oscillation | PD gains too aggressive | Reduce KP_STEER, increase KD_STEER |
| Vehicle hugs one wall | Ultrasonic sensor misread or offset | Verify both sensors return similar values when centered; check wiring |
| Gyro heading drifts | Poor calibration or vibration | Ensure vehicle is perfectly still during calcGyroOffsets(); increase COMP_FILTER_ALPHA |
| Pillars not detected | PixyCam signatures not trained | Retrain signatures in PixyMon under competition lighting |
| Distance estimation wrong | Pixy calibration constants off | Measure actual pillar at known distance, adjust REF_DIST_CM / REF_HEIGHT_PX |
| Motor will not spin | Deadzone too low or wiring | Increase MOTOR_DEADZONE; verify H-Bridge connections |
| Parking fails to align | Magenta detection unreliable | Check Pixy signature 3; verify parking markers are within camera FOV |
| Lap count incorrect | Heading drift or wrong direction | Verify DRIVING_DIRECTION matches actual rotation; check gyro calibration |
| Build errors | Missing libraries | Run pio lib install; verify platformio.ini has all lib_deps |

### Serial Debug Output Format

```
[STATE_NAME] L=<leftCm> R=<rightCm> H=<heading> Lap=<count> Spd=<pwm> Str=<angle> P:s<sig> d=<dist>
```

Debug messages print every 500 ms (configurable via DEBUG_PRINT_MS).

---

*Built for the World Robot Olympiad 2026 Future Engineers category.*