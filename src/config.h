#pragma once

// ═══ DEBUG CONFIG ═══
// Uncomment the line below ONLY for bench testing / development
// This MUST be commented out for competition builds
#define DEBUG_BT_ENABLED
#ifdef DEBUG_BT_ENABLED
#define BT_DEVICE_NAME      "EAGoats_FI_WRO2026_DEBUG"
#define BT_SEND_INTERVAL_MS 200
#endif

// ═══ CHALLENGE CONFIG ═══
#define CHALLENGE_MODE          1       // 0=open, 1=obstacle
#define DRIVING_DIRECTION       1       // 1=CW, -1=CCW

// ═══ PINS ═══
#define TRIG_L              12
#define ECHO_L              14
#define TRIG_R              27
#define ECHO_R              26
#define MOTOR_PWM_PIN       32
#define MOTOR_DIR_A         33
#define MOTOR_DIR_B         25
#define SERVO_PIN           13
#define SDA_PIN             21      // GY-521 (MPU-6050) SDA
#define SCL_PIN             22      // GY-521 (MPU-6050) SCL
#define START_BUTTON_PIN    4

// ═══ ULTRASONIC ═══
#define US_TIMEOUT_US       25000
#define US_MAX_CM           150
#define MEDIAN_WINDOW       5
#define SENSOR_SPACING_CM   15.0f

// ═══ GY-521 / MPU-6050 ═══
#define GYRO_CALIBRATION_MS     3000    // Time to hold still for offset calibration
#define COMP_FILTER_ALPHA       0.95f   // Gyro trust ratio in complementary filter
#define GYRO_DRIFT_THRESHOLD    0.05f   // Ignore yaw rates below this (deg/s)

// ═══ PIXY ═══
#define PIXY_SIG_RED        1
#define PIXY_SIG_GREEN      2
#define PIXY_SIG_MAGENTA    3
#define REF_DIST_CM         30.0f
#define REF_HEIGHT_PX       60.0f
#define PIXY_FRAME_WIDTH    316
#define PIXY_FRAME_CENTER   158

// ═══ PD CONTROLLER ═══
#define KP_STEER            1.5f
#define KD_STEER            0.8f
#define KP_PARK             2.0f
#define KD_PARK             1.0f

// ═══ MOTOR ═══
#define SPEED_MAX           220
#define SPEED_MEDIUM        160
#define SPEED_SLOW          100
#define SPEED_PARK          70
#define SPEED_MIN           60
#define MOTOR_DEADZONE      40
#define RAMP_UP_STEP        5
#define RAMP_DOWN_STEP      10

// ═══ SERVO ═══
#define SERVO_CENTER        90
#define SERVO_MIN_ANGLE     45
#define SERVO_MAX_ANGLE     135
#define MAX_SLEW_RATE       5.0f

// ═══ STATE MACHINE ═══
#define EVADE_TRIGGER_CM        25.0f
#define CENTER_TOLERANCE_CM     8.0f
#define GAP_THRESHOLD_CM        15.0f
#define EVADE_TIMEOUT_MS        1200
#define CORNER_HEADING_DEG      80.0f
#define PARKING_TURN_DEG        85.0f
#define PARKING_PARALLEL_TOL    2.0f
#define WALL_COLLISION_CM       5.0f

// ═══ LOOK-AHEAD ═══
#define REACTION_DIST_FAST      60.0f
#define REACTION_DIST_MEDIUM    40.0f
#define REACTION_DIST_SLOW      25.0f

// ═══ SCHMITT TRIGGER ═══
#define OBSTACLE_SCHMITT_LOW    25.0f
#define OBSTACLE_SCHMITT_HIGH   35.0f
#define GAP_SCHMITT_LOW         40.0f
#define GAP_SCHMITT_HIGH        55.0f

// ═══ TIMING ═══
#define SENSOR_TASK_MS      25
#define CONTROL_LOOP_MS     10
#define DEBUG_PRINT_MS      500

// ═══ LAPS ═══
#define TOTAL_LAPS          3
#define SECTIONS_PER_LAP    8