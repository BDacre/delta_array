#include "variables_and_parameters.h"

// ---------------------------------------------------------
// Motor hardware
// ---------------------------------------------------------
Adafruit_MotorShield MC0 = Adafruit_MotorShield(MC0_ADDR);
Adafruit_MotorShield MC1 = Adafruit_MotorShield(MC1_ADDR);
Adafruit_MotorShield MC2 = Adafruit_MotorShield(MC2_ADDR);

Adafruit_DCMotor *MC0_M1 = MC0.getMotor(1);
Adafruit_DCMotor *MC0_M2 = MC0.getMotor(2);
Adafruit_DCMotor *MC0_M3 = MC0.getMotor(3);
Adafruit_DCMotor *MC0_M4 = MC0.getMotor(4);
Adafruit_DCMotor *MC1_M1 = MC1.getMotor(1);
Adafruit_DCMotor *MC1_M2 = MC1.getMotor(2);
Adafruit_DCMotor *MC1_M3 = MC1.getMotor(3);
Adafruit_DCMotor *MC1_M4 = MC1.getMotor(4);
Adafruit_DCMotor *MC2_M1 = MC2.getMotor(1);
Adafruit_DCMotor *MC2_M2 = MC2.getMotor(2);
Adafruit_DCMotor *MC2_M3 = MC2.getMotor(3);
Adafruit_DCMotor *MC2_M4 = MC2.getMotor(4);

Adafruit_DCMotor* motors[NUM_MOTORS] = {MC0_M1,MC0_M2,MC1_M1,
                                        MC1_M2,MC2_M1,MC2_M2,
                                        MC2_M3,MC2_M4,MC1_M3,
                                        MC1_M4,MC0_M3,MC0_M4};

// ---------------------------------------------------------
// ADC hardware
// ---------------------------------------------------------
Adafruit_ADS1015 ADC0;
Adafruit_ADS1015 ADC1;
Adafruit_ADS1015 ADC2;

Adafruit_ADS1015* adcs[NUM_MOTORS] = {&ADC2, &ADC2, &ADC1,
                                      &ADC1, &ADC0, &ADC0,
                                      &ADC0, &ADC0, &ADC1,
                                      &ADC1, &ADC2, &ADC2};

int channels[NUM_MOTORS] = {0,1,0,
                            1,0,1,
                            2,3,2,
                            3,2,3};

// ---------------------------------------------------------
// Serial comms state
// ---------------------------------------------------------
uint8_t input_cmd[NUM_CHARS];
bool newData = false;

uint16_t ndx = 0;
uint8_t startMarker = 0xA6;
uint8_t endMarker = 0xA7;

// ---------------------------------------------------------
// Trajectory state
// ---------------------------------------------------------
float trajectory[MAX_TRAJ_ROWS][NUM_MOTORS] = {{0.0f}};
int traj_iter = 0;
int traj_rows = 0;
bool go = false;

// ---------------------------------------------------------
// Control loop state
// ---------------------------------------------------------
unsigned long current_arduino_time;
unsigned long last_arduino_time;
float time_elapsed;

float joint_positions[NUM_MOTORS] = {0.0};
float new_joint_positions[NUM_MOTORS] = {0.0};

float position_threshold = 0.0008;
float joint_errors[NUM_MOTORS] = {0.0};
float last_joint_errors[NUM_MOTORS] = {0.0};
float total_joint_errors[NUM_MOTORS] = {0.0};

int motor_val[NUM_MOTORS] = {0};
bool is_movement_done = false;
