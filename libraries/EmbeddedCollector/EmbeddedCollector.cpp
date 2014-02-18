/*
 * Squad Collaborative Robotics
 * Prototype Embedded Collector Bot Code
 * Arduino Mega 2560 R3
 */

#include <EmbeddedCollector.h>

// Placeholders for wheel speeds in rad/s
double L_WheelVelocity;
double R_WheelVelocity;

// Pololu Motor Driver Initialization
// Rx, Tx, Reset Pins
PololuQik2s12v10 pmd(PMD_RX, PMD_TX, PMD_RESET);

// Placeholders for speed commands
double lw_cmd_spd;
double rw_cmd_spd;

// Placeholders for motor commands
double lm_cmd;
double rm_cmd;

// Encoder object initialization
Encoder L_DCMotorEncoder(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN);
Encoder R_DCMotorEncoder(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN);

// Encoder value initialization
long last_encoder_time;

// Left Encoder
long L_EncoderValue;         // Raw Encoder Pulses
float L_EncoderVelocity;     // Radians per second
long L_LastEncoderValue;     // Raw Encoder Pulses
double L_EncoderDelta;       // Raw Encoder Pulses
double L_LastEncoderDelta;   // Raw Encoder Pulses

// Right Encoder
long R_EncoderValue;         // Raw Encoder Pulses
float R_EncoderVelocity;     // Radians per second
long R_LastEncoderValue;     // Raw Encoder Pulses     
double R_EncoderDelta;       // Raw Encoder Pulses
double R_LastEncoderDelta;   // Raw Encoder Pulses

// Low Pass Parameter for encoder velocity
double beta;  

// Speed Controller Declaration
double L_PIDout;
double R_PIDout;

// Input, Output, Setpoint, P, I, D, DIRECT/REVERSE)
PID L_DCMotorPID(&L_WheelVelocity,
               &L_PIDout,
               &lw_cmd_spd,
               0.15,
               0.0,
               0.015,
               REVERSE);

PID R_DCMotorPID(&R_WheelVelocity,
               &R_PIDout,
               &rw_cmd_spd,
               0.15,
               0.0,
               0.015,
               REVERSE);

// Mode Setter -> 0 for Safe Mode
//             -> 1 for Normal Functionality
int mode = 0;

void EmbeddedCollector::initialize(){

  // Initialize Encoders
  initEncoders();

  // Initialize Motors
  initMotors();

}

void EmbeddedCollector::initEncoders(){

  // Encoder value initialization
  last_encoder_time = micros();  // not the best way to get this done... should check existence in Update Wheel Function

  // Left Encoder
  L_EncoderValue = 0;
  L_EncoderVelocity = 0; // Radians per second
  L_LastEncoderValue = 0;
  L_EncoderDelta = 0; 
  L_LastEncoderDelta = 0; 

  // Right Encoder
  R_EncoderValue = 0;
  R_EncoderVelocity = 0; // Radians per second
  R_LastEncoderValue = 0;
  R_EncoderDelta = 0; 
  R_LastEncoderDelta = 0; 

  // Low Pass Parameter for encoder velocity
  beta = 0.0;

  // Placeholders for wheel speeds in rad/s
  L_WheelVelocity = 0.0;
  R_WheelVelocity = 0.0;

}  

void EmbeddedCollector::initMotors(){

  // Placeholders for motor commands
  lm_cmd = 0;
  rm_cmd = 0;

  // Placeholders for speed commands
  lw_cmd_spd = 0;
  rw_cmd_spd = 0;

  // Speed Controller Initialization
  L_PIDout = 0;
  R_PIDout = 0;

  // PID Setup
  L_DCMotorPID.SetMode(AUTOMATIC);
  L_DCMotorPID.SetOutputLimits(-127, 127);
  L_DCMotorPID.SetSampleTime(20);
  R_DCMotorPID.SetMode(AUTOMATIC);
  R_DCMotorPID.SetOutputLimits(-127, 127);
  R_DCMotorPID.SetSampleTime(20);

  // Motor Driver Init
  pmd.init();

}


void EmbeddedCollector::updateEncoderReading() {
  // Get raw ticks
  L_EncoderValue = L_DCMotorEncoder.read();
  R_EncoderValue = R_DCMotorEncoder.read();
  
  // Current time in micro seconds
  long now = micros();
  
  long dt = (now - last_encoder_time);
  
  // Get position in revolutions (includes multiple revolutions) with low pass filter
  L_EncoderDelta = (double)(L_EncoderValue - L_LastEncoderValue) * (1-beta) + L_LastEncoderDelta * beta;
  R_EncoderDelta = (double)(R_EncoderValue - R_LastEncoderValue) * (1-beta) + R_LastEncoderDelta * beta;
  
  // Get velocity in revs/sec
  L_EncoderVelocity = 1000000.0/dt * L_EncoderDelta/TICKS_PER_REVOLUTION;
  R_EncoderVelocity = 1000000.0/dt * R_EncoderDelta/TICKS_PER_REVOLUTION;
  
  // Convert to rad/sec
  L_WheelVelocity = L_EncoderVelocity * 2 * PI;
  R_WheelVelocity = R_EncoderVelocity * 2 * PI;

  // Update previous values
  L_LastEncoderValue = L_EncoderValue;
  R_LastEncoderValue = R_EncoderValue;

  last_encoder_time = now;

  L_LastEncoderDelta = L_EncoderDelta;
  R_LastEncoderDelta = R_EncoderDelta;

}

float* EmbeddedCollector::getWheelVels(){

  float * pointer;
  float out[1];

  updateEncoderReading();
  out[0] = L_WheelVelocity;
  out[1] = R_WheelVelocity;
  pointer = out;
  return pointer;

}

void EmbeddedCollector::setMode(int cmd_mode){

  mode = cmd_mode;

}

void EmbeddedCollector::setMotorCommands(float lw_cmd, float rw_cmd){

  lw_cmd_spd = lw_cmd;
  rw_cmd_spd = rw_cmd;

}

void EmbeddedCollector::controlLoop(){

  // Get Freshest Values
  updateEncoderReading();

  // Compute New Control Values
  L_DCMotorPID.Compute();
  R_DCMotorPID.Compute();

  // Update motor command
  lm_cmd += L_PIDout;
  rm_cmd += R_PIDout;

  // Write Speeds to motor driver
  pmd.setSpeeds(lm_cmd, rm_cmd);

}

void EmbeddedCollector::safeControlLoop(){
  // Keep PID from freaking out on resume with 0 command speeds
  lw_cmd_spd = 0;
  rw_cmd_spd = 0;

  // Get Freshest Values
  updateEncoderReading();

  // Compute New Control Values
  L_DCMotorPID.Compute();
  R_DCMotorPID.Compute();

  lm_cmd = 0;
  rm_cmd = 0;

  // Stop motors directly
  pmd.setSpeeds(0, 0);
}

void EmbeddedCollector::spin(){

  // If not in safe mode, can move motors
  if(mode){
    // Handle motor commands
    controlLoop();
  }
  else{
    safeControlLoop();
  }
}
