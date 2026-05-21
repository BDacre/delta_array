#if !defined(VARIABLES_AND_PARAMETERS_H)
#define VARIABLES_AND_PARAMETERS_H

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_ADS1X15.h>

// ---------------------------------------------------------
// Configuration
// ---------------------------------------------------------
#define NUM_MOTORS 12
#define MY_ID 9

#define SERIAL_BAUD 57600
#define INIT_MOTOR_SPEED 150

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

// ---------------------------------------------------------
// Serial buffer
// ---------------------------------------------------------
#define NUM_CHARS 128

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

extern boolean recvInProgress;
extern byte ndx;
extern char startMarker;
extern char endMarker;

// ---------------------------------------------------------
// Control loop state
// ---------------------------------------------------------
extern unsigned long current_arduino_time;
extern unsigned long last_arduino_time;
extern float time_elapsed;

extern float joint_positions[NUM_MOTORS];
extern float new_joint_positions[NUM_MOTORS];

extern float position_threshold;
extern float joint_errors[NUM_MOTORS];
extern float last_joint_errors[NUM_MOTORS];
extern float total_joint_errors[NUM_MOTORS];

extern int motor_val[NUM_MOTORS];
extern bool is_movement_done;

#endif // VARIABLES_AND_PARAMETERS_H
