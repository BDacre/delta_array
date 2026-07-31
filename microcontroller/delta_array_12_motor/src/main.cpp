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
#include "variables_and_parameters.h"


void readJointPositions();
bool runPidStep();
void releaseAllMotors();
void resetPidState();
void loadTrajectoryRow(int row);
void recvWithStartEndMarkers();
bool decodeNanopbData();
static void sendFramedResponse(const DeltaMessage &response);
static void sendAck(AckStatus status);
static bool handleStatus(const StatusFrame &status);
static bool handleJoint(const JointFrame &joint);
static uint16_t crc16_ccitt(const uint8_t *data, size_t len);
static uint32_t computeChipId();

// If a frame stalls mid-flight (cable yank, host crash), reset the receive
// state machine rather than blocking on the missing bytes forever.
static const unsigned long RX_INTERBYTE_TIMEOUT_MS = 100UL;

enum RxState : uint8_t {
  RX_WAIT_START,
  RX_LEN_LO,
  RX_LEN_HI,
  RX_PAYLOAD,
  RX_CRC_LO,
  RX_CRC_HI,
  RX_END,
};

static RxState rx_state = RX_WAIT_START;
static uint16_t rx_expected_len = 0;
static uint16_t rx_received_len = 0;
static uint16_t rx_crc_received = 0;
static unsigned long rx_last_byte_ms = 0;


// Derive a stable per-board id from the SAMD21's 128-bit factory serial number.
// The full 16 bytes don't fit the int32 wire id field, so hash them (FNV-1a)
// into 32 bits, mask to 31 bits so the id is always a positive int32, and force
// non-zero so a derived id can never collide with BROADCAST_ID (0). Same chip
// always yields the same id; different chips effectively never collide.
static uint32_t computeChipId() {
  const uint32_t serial_addrs[4] = {
    SAMD_SERIAL_WORD0, SAMD_SERIAL_WORD1, SAMD_SERIAL_WORD2, SAMD_SERIAL_WORD3,
  };
  uint32_t hash = 2166136261UL;  // FNV-1a offset basis
  for (int w = 0; w < 4; w++) {
    uint32_t word = *(volatile uint32_t *)serial_addrs[w];
    for (int b = 0; b < 4; b++) {
      hash ^= (word >> (8 * b)) & 0xFF;
      hash *= 16777619UL;         // FNV-1a prime
    }
  }
  hash &= 0x7FFFFFFFUL;           // keep within positive int32 range
  if (hash == 0) hash = 1;        // reserve 0 for BROADCAST_ID
  return hash;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial)
    delay(10);

  // Establish this board's identity before servicing any commands.
  my_id = computeChipId();

  // Initialize motor shields and ADCs
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
}

void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
    decodeNanopbData();
    newData = false;
    ndx = 0;
  }

  if (ctrl_mode == CTRL_IDLE) return;

  // CTRL_OPENLOOP: one motor is held at a fixed PWM for diagnostics. The PID is
  // bypassed entirely; we only enforce the safety auto-release deadline. The
  // subtraction compare is rollover-safe.
  if (ctrl_mode == CTRL_OPENLOOP) {
    if ((int32_t)(millis() - openloop_deadline) >= 0) {
      releaseAllMotors();
      openloop_motor = -1;
      ctrl_mode = CTRL_IDLE;
    }
    return;
  }

  bool reached = runPidStep();

  if (ctrl_mode == CTRL_HOLD) {
    if (reached || (millis() - target_start_ms) > MOVE_TIMEOUT_MS) {
      releaseAllMotors();
      ctrl_mode = CTRL_IDLE;
    }
    return;
  }

  // CTRL_TRAJ: advance to next row once joints settle, or on timeout
  // (timeout per-row so a stuck point doesn't strand the whole trajectory).
  if (reached || (millis() - target_start_ms) > MOVE_TIMEOUT_MS) {
    traj_iter++;
    if (traj_iter < traj_rows) {
      loadTrajectoryRow(traj_iter);
    } else {
      releaseAllMotors();
      ctrl_mode = CTRL_IDLE;
    }
  }
}

void readJointPositions(){
  for(int i = 0; i < NUM_MOTORS; i++){
    motor_val[i] = adcs[i]->readADC_SingleEnded(channels[i]);
    joint_positions[i] = motor_val[i] * ADC_TO_POSITION;
  }
}

// One PID iteration toward new_joint_positions[]. Returns true when every
// joint is within position_threshold of its target. Caller decides what to
// do next (release, advance trajectory, time out).
bool runPidStep(){
  current_arduino_time = millis();
  time_elapsed = float(current_arduino_time - last_arduino_time) / 1000.0;
  // First step after a new target: dt would be ~0 and blow up the D term.
  // Skip derivative this tick by clamping dt to a non-zero floor.
  if (time_elapsed <= 0.0f) time_elapsed = 0.001f;

  readJointPositions();
  bool reached = true;
  for(int i = 0; i < NUM_MOTORS; i++){
    joint_errors[i] = joint_positions[i] - new_joint_positions[i];
    float pid = KP * joint_errors[i] + KI * total_joint_errors[i] + KD * (joint_errors[i] - last_joint_errors[i]) / time_elapsed;
    if(joint_errors[i] > deadband[i]){
      // Static feedforward bias lifts the drive past breakaway so small errors
      // don't stall below the friction floor; clamp the total to PWM_MAX.
      int motor_speed = (int)(min(max(0.0, pid), 1.0) * PWM_MAX) + bias_back[i];
      if (motor_speed > (int)PWM_MAX) motor_speed = (int)PWM_MAX;
      reached = false;
      motors[i]->setSpeed(motor_speed);
      motors[i]->run(BACKWARD);
      applied_pwm[i] = motor_speed;   // BACKWARD -> positive (matches sign convention)
      total_joint_errors[i] += joint_errors[i];
    }
    else if(joint_errors[i] < -deadband[i]){
      int motor_speed = (int)(min(max(-1.0, pid), 0.0) * -PWM_MAX) + bias_fwd[i];
      if (motor_speed > (int)PWM_MAX) motor_speed = (int)PWM_MAX;
      reached = false;
      motors[i]->setSpeed(motor_speed);
      motors[i]->run(FORWARD);
      applied_pwm[i] = -motor_speed;  // FORWARD -> negative
      total_joint_errors[i] += joint_errors[i];
    }
    else{
      motors[i]->setSpeed(0);
      motors[i]->run(RELEASE);
      applied_pwm[i] = 0;
      total_joint_errors[i] = 0.0;
    }
    last_joint_errors[i] = joint_errors[i];
  }
  last_arduino_time = current_arduino_time;
  return reached;
}

void releaseAllMotors(){
  for(int i = 0; i < NUM_MOTORS; i++){
    motors[i]->setSpeed(0);
    motors[i]->run(RELEASE);
    applied_pwm[i] = 0;
  }
}

// Clear integral/derivative carry-over before driving toward a new target,
// otherwise stale error history kicks the first PID step in the wrong
// direction.
void resetPidState(){
  for(int i = 0; i < NUM_MOTORS; i++){
    last_joint_errors[i] = 0.0f;
    total_joint_errors[i] = 0.0f;
  }
  last_arduino_time = millis();
  target_start_ms = last_arduino_time;
}

void loadTrajectoryRow(int row){
  for (int j = 0; j < NUM_MOTORS; j++){
    new_joint_positions[j] = trajectory[row][j];
  }
  resetPidState();
}

//Cyclic redundancy check (CRC-16/CCITT-FALSE(. Checker for data integrity in serial communication. Uses polynomial 0x1021 and initial value 0xFFFF.)
static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i]) << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

void recvWithStartEndMarkers() {
  // Drop a stalled frame if the host went quiet mid-transmission.
  if (rx_state != RX_WAIT_START &&
      (millis() - rx_last_byte_ms) > RX_INTERBYTE_TIMEOUT_MS) {
    rx_state = RX_WAIT_START;
  }

  while (Serial.available() > 0 && newData == false) {
    uint8_t rc = (uint8_t)Serial.read();
    rx_last_byte_ms = millis();

    switch (rx_state) {
      case RX_WAIT_START:
        if (rc == startMarker) {
          rx_state = RX_LEN_LO;
        }
        break;

      case RX_LEN_LO:
        rx_expected_len = rc;
        rx_state = RX_LEN_HI;
        break;

      case RX_LEN_HI:
        rx_expected_len |= ((uint16_t)rc) << 8;
        if (rx_expected_len == 0 || rx_expected_len > NUM_CHARS) {
          // Implausible length: abandon and resync.
          rx_state = RX_WAIT_START;
        } else {
          rx_received_len = 0;
          rx_state = RX_PAYLOAD;
        }
        break;

      case RX_PAYLOAD:
        input_cmd[rx_received_len++] = rc;
        if (rx_received_len >= rx_expected_len) {
          rx_state = RX_CRC_LO;
        }
        break;

      case RX_CRC_LO:
        rx_crc_received = rc;
        rx_state = RX_CRC_HI;
        break;

      case RX_CRC_HI:
        rx_crc_received |= ((uint16_t)rc) << 8;
        rx_state = RX_END;
        break;

      case RX_END:
        if (rc == endMarker &&
            crc16_ccitt(input_cmd, rx_expected_len) == rx_crc_received) {
          ndx = rx_expected_len;
          newData = true;
        }
        rx_state = RX_WAIT_START;
        break;
    }
  }
}

static void sendFramedResponse(const DeltaMessage &response){
  uint8_t out_buf[RESPONSE_BUF_BYTES];
  pb_ostream_t ostream = pb_ostream_from_buffer(out_buf, sizeof(out_buf));
  if (!pb_encode(&ostream, DeltaMessage_fields, &response)) return;
  uint16_t len = (uint16_t)ostream.bytes_written;
  uint16_t crc = crc16_ccitt(out_buf, len);
  uint8_t header[3] = { startMarker, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
  uint8_t trailer[3] = { (uint8_t)(crc & 0xFF), (uint8_t)(crc >> 8), endMarker };
  Serial.write(header, sizeof(header));
  Serial.write(out_buf, len);
  Serial.write(trailer, sizeof(trailer));
}

// Acceptance ACK for a JointFrame command. Sent only by the addressed board,
// only after id has been validated. Does NOT signal completion — completion
// is still polled via StatusFrame.done_req.
static void sendAck(AckStatus status){
  DeltaMessage response = DeltaMessage_init_zero;
  response.id = my_id;
  response.which_payload = DeltaMessage_ack_tag;
  response.payload.ack.status = status;
  sendFramedResponse(response);
}

static bool handleStatus(const StatusFrame &status){
  DeltaMessage response = DeltaMessage_init_zero;
  response.id = my_id;
  response.which_payload = DeltaMessage_status_tag;
  switch (status.which_kind){
    case StatusFrame_pose_req_tag: {
      readJointPositions();
      response.payload.status.which_kind = StatusFrame_pose_resp_tag;
      response.payload.status.kind.pose_resp.joint_pos_count = NUM_MOTORS;
      for (int i = 0; i < NUM_MOTORS; i++){
        response.payload.status.kind.pose_resp.joint_pos[i] = joint_positions[i];
      }
      sendFramedResponse(response);
      return false;
    }
    case StatusFrame_done_req_tag: {
      response.payload.status.which_kind = StatusFrame_done_resp_tag;
      response.payload.status.kind.done_resp.done = (ctrl_mode == CTRL_IDLE);
      sendFramedResponse(response);
      return false;
    }
    case StatusFrame_id_req_tag: {
      // Whoami: report this board's derived id. The DeltaMessage.id above
      // already carries my_id, so the host learns the id even when it asked
      // via the broadcast id; id_resp.id restates it explicitly.
      response.payload.status.which_kind = StatusFrame_id_resp_tag;
      response.payload.status.kind.id_resp.id = my_id;
      sendFramedResponse(response);
      return false;
    }
    case StatusFrame_telemetry_req_tag: {
      // Diagnostics: report per-motor position, PID error, and last applied PWM.
      // Read-only, safe in any control mode. error/pwm reflect the most recent
      // control step (updated by runPidStep / the open-loop drive).
      readJointPositions();
      response.payload.status.which_kind = StatusFrame_telemetry_resp_tag;
      TelemetryResponse &t = response.payload.status.kind.telemetry_resp;
      t.position_count = NUM_MOTORS;
      t.error_count = NUM_MOTORS;
      t.pwm_count = NUM_MOTORS;
      for (int i = 0; i < NUM_MOTORS; i++){
        t.position[i] = joint_positions[i];
        t.error[i] = joint_errors[i];
        t.pwm[i] = applied_pwm[i];
      }
      sendFramedResponse(response);
      return false;
    }
    default:
      return false;
  }
}

static bool handleJoint(const JointFrame &joint){
  switch (joint.which_kind){
    case JointFrame_move_tag: {
      const MoveCommand &cmd = joint.kind.move;
      if (cmd.joint_pos_count != NUM_MOTORS) {
        sendAck(AckStatus_ACK_VALIDATION_FAIL);
        return false;
      }
      for (int i = 0; i < NUM_MOTORS; i++){
        new_joint_positions[i] = cmd.joint_pos[i];
      }
      resetPidState();
      ctrl_mode = CTRL_HOLD;
      sendAck(AckStatus_ACK_OK);
      return true;
    }
    case JointFrame_traj_tag: {
      const TrajectoryCommand &cmd = joint.kind.traj;
      int n = cmd.joint_pos_count;
      if (n <= 0 || n > MAX_TRAJ_FLOATS || (n % NUM_MOTORS) != 0) {
        sendAck(AckStatus_ACK_VALIDATION_FAIL);
        return false;
      }
      traj_rows = n / NUM_MOTORS;
      for (int i = 0; i < traj_rows; i++){
        for (int j = 0; j < NUM_MOTORS; j++){
          trajectory[i][j] = cmd.joint_pos[i * NUM_MOTORS + j];
        }
      }
      traj_iter = 0;
      loadTrajectoryRow(0);
      ctrl_mode = CTRL_TRAJ;
      sendAck(AckStatus_ACK_OK);
      return true;
    }
    case JointFrame_reset_tag: {
      for (int i = 0; i < NUM_MOTORS; i++){
        new_joint_positions[i] = RESET_POSITION;
      }
      resetPidState();
      ctrl_mode = CTRL_HOLD;
      sendAck(AckStatus_ACK_OK);
      return true;
    }
    case JointFrame_stop_tag: {
      releaseAllMotors();
      openloop_motor = -1;
      ctrl_mode = CTRL_IDLE;
      sendAck(AckStatus_ACK_OK);
      return true;
    }
    case JointFrame_set_pwm_tag: {
      // Diagnostics: drive one motor open-loop at a fixed PWM, PID bypassed.
      const SetPwmCommand &cmd = joint.kind.set_pwm;
      if (cmd.motor_index >= (uint32_t)NUM_MOTORS) {
        sendAck(AckStatus_ACK_VALIDATION_FAIL);
        return false;
      }
      int idx = (int)cmd.motor_index;
      // Clamp magnitude to PWM_MAX; sign of pwm selects direction.
      int mag = (cmd.pwm >= 0) ? cmd.pwm : -cmd.pwm;
      if (mag > (int)PWM_MAX) mag = (int)PWM_MAX;
      // Clamp the auto-release window; 0 -> default. Never longer than the move
      // timeout, so a lost host can't leave a motor driven indefinitely.
      unsigned long dur = cmd.duration_ms;
      if (dur == 0) dur = OPENLOOP_DEFAULT_MS;
      if (dur > MOVE_TIMEOUT_MS) dur = MOVE_TIMEOUT_MS;
      // Release everything first so only the target motor moves and it never
      // fights the PID, then drive the one motor.
      releaseAllMotors();
      if (mag > 0) {
        motors[idx]->setSpeed(mag);
        motors[idx]->run(cmd.pwm >= 0 ? BACKWARD : FORWARD);
        applied_pwm[idx] = (cmd.pwm >= 0) ? mag : -mag;
      }
      openloop_motor = idx;
      openloop_deadline = millis() + dur;
      ctrl_mode = CTRL_OPENLOOP;
      sendAck(AckStatus_ACK_OK);
      return true;
    }
    case JointFrame_set_config_tag: {
      // Runtime per-motor tuning (deadband + static feedforward bias). Only the
      // fields present in the message are applied (proto3 optional / has_*);
      // absent fields keep their current value. motor_index absent => all motors.
      // Does not touch the control mode, so it is safe to send while idle or
      // between moves (e.g. pushing a board's calibration at startup).
      const SetConfigCommand &cmd = joint.kind.set_config;
      int lo = 0, hi = NUM_MOTORS - 1;
      if (cmd.has_motor_index) {
        if (cmd.motor_index >= (uint32_t)NUM_MOTORS) {
          sendAck(AckStatus_ACK_VALIDATION_FAIL);
          return false;
        }
        lo = hi = (int)cmd.motor_index;
      }
      for (int i = lo; i <= hi; i++) {
        if (cmd.has_deadband && cmd.deadband >= 0.0f) deadband[i] = cmd.deadband;
        if (cmd.has_bias_fwd)  bias_fwd[i]  = constrain(cmd.bias_fwd, 0, (int)PWM_MAX);
        if (cmd.has_bias_back) bias_back[i] = constrain(cmd.bias_back, 0, (int)PWM_MAX);
      }
      sendAck(AckStatus_ACK_OK);
      return true;
    }
    default:
      sendAck(AckStatus_ACK_UNKNOWN_COMMAND);
      return false;
  }
}

bool decodeNanopbData(){
  DeltaMessage message = DeltaMessage_init_zero;
  pb_istream_t istream = pb_istream_from_buffer(input_cmd, ndx);
  if (!pb_decode(&istream, DeltaMessage_fields, &message)) return false;
  // Accept frames addressed to this board's own id or to the backdoor/broadcast
  // id (so a board can be driven before its id is known). Responses still carry
  // my_id, never the broadcast id. my_id is always a positive int32 (masked to
  // 31 bits), so the unsigned compare is safe.
  if ((uint32_t)message.id != my_id && message.id != BROADCAST_ID) return false;

  switch (message.which_payload){
    case DeltaMessage_status_tag: return handleStatus(message.payload.status);
    case DeltaMessage_joint_tag:  return handleJoint(message.payload.joint);
    default: return false;
  }
}
