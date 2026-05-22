#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_ADS1X15.h>
#include "delta_array.pb.h"
#include "pb_common.h"
#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include <math.h>
#include "varaibles_and_parameters.h"

void readJointPositions();
void writeJointPositions();
void resetJoints();
void stop();
void recvWithStartEndMarkers();
bool decodeNanopbData();
void executeTrajectory();


void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial)
    delay(10);

  MC0.begin();
  MC1.begin();
  MC2.begin();

  ADC2.begin(ADC2_ADDR);
  ADC1.begin(ADC1_ADDR);
  ADC0.begin(ADC0_ADDR);

  ADC2.setGain(GAIN_ONE);
  ADC1.setGain(GAIN_ONE);
  ADC0.setGain(GAIN_ONE);

  for(int i=0; i<NUM_MOTORS; i++){
    motors[i]->setSpeed(INIT_MOTOR_SPEED);
    motors[i]->run(RELEASE);
    delay(10);
  }

  readJointPositions();

  Serial.print("READY id=");
  Serial.println(MY_ID);
}

void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
    if (decodeNanopbData()){
      if (!go) {
        writeJointPositions();
      }
    }
    newData = false;
    ndx = 0;
  }
  if (go) {
    executeTrajectory();
  }
}

void readJointPositions(){
  for(int i = 0; i < NUM_MOTORS; i++){
    motor_val[i] = adcs[i]->readADC_SingleEnded(channels[i]);
    joint_positions[i] = motor_val[i] * ADC_TO_POSITION;
  }
}

void writeJointPositions(){
  bool reached_point = false;
  unsigned long move_start_time = millis();
  last_arduino_time = move_start_time;
  is_movement_done = false;
  while (!reached_point && (millis() - move_start_time) < MOVE_TIMEOUT_MS){
    current_arduino_time = millis();
    time_elapsed = float(current_arduino_time - last_arduino_time) / 1000.0;
    readJointPositions();
    reached_point = true;
    for(int i = 0; i < NUM_MOTORS; i++){
      joint_errors[i] = joint_positions[i] - new_joint_positions[i];
      float pid = KP * joint_errors[i] + KI * total_joint_errors[i] + KD * (joint_errors[i] - last_joint_errors[i]) / time_elapsed;
      if(joint_errors[i] > position_threshold){
        int motor_speed = (int)(min(max(0.0, pid), 1.0) * PWM_MAX);
        reached_point = false;
        motors[i]->setSpeed(motor_speed);
        motors[i]->run(BACKWARD);
        total_joint_errors[i] += joint_errors[i];
      }
      else if(joint_errors[i] < -position_threshold){
        int motor_speed = (int)(min(max(-1.0, pid), 0.0) * -PWM_MAX);
        reached_point = false;
        motors[i]->setSpeed(motor_speed);
        motors[i]->run(FORWARD);
        total_joint_errors[i] += joint_errors[i];
      }
      else{
        motors[i]->setSpeed(0);
        motors[i]->run(RELEASE);
        total_joint_errors[i] = 0.0;
      }
      last_joint_errors[i] = joint_errors[i];
    }
    last_arduino_time = current_arduino_time;
  }
  for(int i = 0; i < NUM_MOTORS; i++){
    motors[i]->setSpeed(0);
    motors[i]->run(RELEASE);
    total_joint_errors[i] = 0.0;
  }
  is_movement_done = true;
}

void resetJoints(){
  for(int i = 0; i < NUM_MOTORS; i++){
    new_joint_positions[i] = 0.0;
  }
  writeJointPositions();
}

void stop(){
  for(int i = 0; i < NUM_MOTORS; i++){
    motors[i]->run(RELEASE);
  }
}

void recvWithStartEndMarkers() {
  byte rc;
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
        input_cmd[ndx] = rc;
        ndx++;
        if (ndx >= NUM_CHARS) {
          ndx = NUM_CHARS - 1;
        }
      }
      else {
        input_cmd[ndx] = '\0';
        recvInProgress = false;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

bool decodeNanopbData(){
  DeltaMessage message = DeltaMessage_init_zero;
  pb_istream_t istream = pb_istream_from_buffer(input_cmd, ndx);
  bool ret = pb_decode(&istream, DeltaMessage_fields, &message);
  if (message.id == MY_ID){
    if (message.request_done_state || message.request_joint_pose){
      DeltaMessage response = DeltaMessage_init_zero;
      response.id = MY_ID;
      if (message.request_joint_pose){
        readJointPositions();
        response.joint_pos_count = NUM_MOTORS;
        for (int i = 0; i < NUM_MOTORS; i++){
          response.joint_pos[i] = joint_positions[i];
        }
      }
      uint8_t out_buf[256];
      pb_ostream_t ostream = pb_ostream_from_buffer(out_buf, sizeof(out_buf));
      pb_encode(&ostream, DeltaMessage_fields, &response);
      Serial.write((uint8_t)startMarker);
      Serial.write(out_buf, ostream.bytes_written);
      Serial.write((uint8_t)endMarker);
      return false;
    }
    else if (message.reset){
      for (int i=0; i<NUM_MOTORS; i++){
        new_joint_positions[i] = RESET_POSITION;
      }
      go = false;
    }
    else{
      int n = message.joint_pos_count;
      if (n == NUM_MOTORS){
        for (int i=0; i<NUM_MOTORS; i++){
          new_joint_positions[i] = message.joint_pos[i];
        }
        go = false;
      }
      else if (n > NUM_MOTORS && n <= MAX_TRAJ_FLOATS && (n % NUM_MOTORS) == 0){
        traj_rows = n / NUM_MOTORS;
        for (int i=0; i<traj_rows; i++){
          for (int j=0; j<NUM_MOTORS; j++){
            trajectory[i][j] = message.joint_pos[i*NUM_MOTORS + j];
          }
        }
        traj_iter = 0;
        go = true;
      }
      else {
        ret = false;
      }
    }
  }
  else{
    ret = false;
  }
  return ret;
}

void executeTrajectory(){
  if (traj_iter < traj_rows){
    for (int j=0; j<NUM_MOTORS; j++){
      new_joint_positions[j] = trajectory[traj_iter][j];
    }
    writeJointPositions();
    traj_iter++;
  }
  else {
    go = false;
  }
}
