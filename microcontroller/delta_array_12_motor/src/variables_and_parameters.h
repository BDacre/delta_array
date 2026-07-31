#if !defined(VARIABLES_AND_PARAMETERS_H)
#define VARIABLES_AND_PARAMETERS_H

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_ADS1X15.h>

// ---------------------------------------------------------
// Configuration
// ---------------------------------------------------------
#define NUM_MOTORS 12

// Board id is no longer hard-coded. Each board derives a unique id at boot from
// the SAMD21 factory serial number (see my_id / computeChipId in main.cpp), so
// the same firmware image runs on every board. my_id is set in setup().
extern uint32_t my_id;

// Backdoor / broadcast id. A board also accepts frames addressed to this id, so
// it can be controlled before its real id is known (e.g. discovery, bench
// maintenance). Replies are always stamped with the board's real my_id, never
// the broadcast id, so any reply reveals the true id. Guaranteed distinct from
// any derived id because computeChipId() never returns 0.
#define BROADCAST_ID 0

// SAMD21 (Cortex-M0+) 128-bit unique serial number word addresses. The four
// words are non-contiguous in the datasheet's memory map.
#define SAMD_SERIAL_WORD0 0x0080A00C
#define SAMD_SERIAL_WORD1 0x0080A040
#define SAMD_SERIAL_WORD2 0x0080A044
#define SAMD_SERIAL_WORD3 0x0080A048

#define SERIAL_BAUD 57600
#define INIT_MOTOR_SPEED 150

// ---------------------------------------------------------
// Trajectory buffer
// ---------------------------------------------------------
#define MAX_TRAJ_ROWS 20
#define MAX_TRAJ_FLOATS (MAX_TRAJ_ROWS * NUM_MOTORS)

// ---------------------------------------------------------
// I2C addresses
// ---------------------------------------------------------
#define MC0_ADDR 0x62
#define MC1_ADDR 0x60
#define MC2_ADDR 0x61

#define ADC0_ADDR 0x48
#define ADC1_ADDR 0x49
#define ADC2_ADDR 0x4A

// ---------------------------------------------------------
// Control parameters
// ---------------------------------------------------------
#define KP 300.0f
#define KI 0.1f
#define KD 3.75f

#define ADC_TO_POSITION 0.00006f
#define PWM_MAX 255.0f
#define RESET_POSITION 0.05f

// Safety: abort the active move (CTRL_HOLD) if PID can't settle within this
// window. Prevents a single unreachable target from holding the motors driven
// forever. Measured from the time the target was accepted. Trajectory rows
// reset the timer on every advance.
#define MOVE_TIMEOUT_MS 5000UL

// ---------------------------------------------------------
// Serial buffer
// ---------------------------------------------------------
// Sized to hold a max-length trajectory: 240 floats packed-encoded (~5 B/float),
// plus header and bool fields. ~1200 B leaves headroom for the encoded message.
#define NUM_CHARS 2048 //Max is (DeltaMessage_size + 16), 1217 for 12*20 but leave headroom for safety

// Outbound responses. pose_resp/done_resp are tiny, but telemetry_resp carries
// three 12-element arrays (position + error + pwm) ~ 200 B encoded, so give it
// headroom. Kept separate from NUM_CHARS so we don't burn 2 KB of stack per
// response.
#define RESPONSE_BUF_BYTES 384

// ---------------------------------------------------------
// Motor hardware
// ---------------------------------------------------------
extern Adafruit_MotorShield MC0;
extern Adafruit_MotorShield MC1;
extern Adafruit_MotorShield MC2;

extern Adafruit_DCMotor *MC0_M1;
extern Adafruit_DCMotor *MC0_M2;
extern Adafruit_DCMotor *MC0_M3;
extern Adafruit_DCMotor *MC0_M4;
extern Adafruit_DCMotor *MC1_M1;
extern Adafruit_DCMotor *MC1_M2;
extern Adafruit_DCMotor *MC1_M3;
extern Adafruit_DCMotor *MC1_M4;
extern Adafruit_DCMotor *MC2_M1;
extern Adafruit_DCMotor *MC2_M2;
extern Adafruit_DCMotor *MC2_M3;
extern Adafruit_DCMotor *MC2_M4;

extern Adafruit_DCMotor* motors[NUM_MOTORS];

// ---------------------------------------------------------
// ADC hardware
// ---------------------------------------------------------
extern Adafruit_ADS1015 ADC0;
extern Adafruit_ADS1015 ADC1;
extern Adafruit_ADS1015 ADC2;

extern Adafruit_ADS1015* adcs[NUM_MOTORS];

extern int channels[NUM_MOTORS];

// ---------------------------------------------------------
// Serial comms state
// ---------------------------------------------------------
extern uint8_t input_cmd[NUM_CHARS];
extern bool newData;

extern uint16_t ndx;
extern uint8_t startMarker;
extern uint8_t endMarker;

// ---------------------------------------------------------
// Control mode
// ---------------------------------------------------------
// CTRL_IDLE: motors released, no active target.
// CTRL_HOLD: driving toward a single target (MoveCommand / ResetCommand).
// CTRL_TRAJ: stepping through trajectory[] rows; advances to the next row
//            once all joints settle within POSITION_THRESHOLD.
// CTRL_OPENLOOP: diagnostics only — one motor driven at a fixed PWM (PID bypassed),
//            auto-released at openloop_deadline. Set by SetPwmCommand.
enum ControlMode : uint8_t {
  CTRL_IDLE = 0,
  CTRL_HOLD = 1,
  CTRL_TRAJ = 2,
  CTRL_OPENLOOP = 3,
};

extern ControlMode ctrl_mode;
extern unsigned long target_start_ms;

// ---------------------------------------------------------
// Open-loop (diagnostics) state
// ---------------------------------------------------------
// Default auto-release window for a SetPwmCommand with duration_ms == 0.
#define OPENLOOP_DEFAULT_MS 500UL
extern int openloop_motor;              // motor being driven, or -1 when none
extern unsigned long openloop_deadline; // millis() at which to auto-release

// ---------------------------------------------------------
// Trajectory state
// ---------------------------------------------------------
extern float trajectory[MAX_TRAJ_ROWS][NUM_MOTORS];
extern int traj_iter;
extern int traj_rows;

// ---------------------------------------------------------
// Control loop state
// ---------------------------------------------------------
extern unsigned long current_arduino_time;
extern unsigned long last_arduino_time;
extern float time_elapsed;

extern float joint_positions[NUM_MOTORS];
extern float new_joint_positions[NUM_MOTORS];

extern float POSITION_THRESHOLD;
extern float joint_errors[NUM_MOTORS];
extern float last_joint_errors[NUM_MOTORS];
extern float total_joint_errors[NUM_MOTORS];

extern int motor_val[NUM_MOTORS];

// Last signed PWM applied to each motor (-PWM_MAX..PWM_MAX; 0 = released).
// Recorded by both the PID (runPidStep) and the open-loop drive so telemetry
// reflects what the motors are actually doing.
extern int applied_pwm[NUM_MOTORS];

#endif // VARIABLES_AND_PARAMETERS_H
