/*
 * Squad Collaborative Robotics
 * Prototype Embedded Collector Bot Code
 * Arduino Mega 2560 R3
 */

#include <EmbeddedCollector.h>

// Global Nodehandle object
ros::NodeHandle_<ArduinoHardware, 5, 5, 256, 256> nh;

// Wheel Speed Messages
std_msgs::Float32 l_wheel_msg;
std_msgs::Float32 r_wheel_msg;

// Motors Command Message
scr_proto::SpeedCommand motor_msg;

// Initialize Publishers
ros::Publisher left_wheel_pub("lw_speed", &l_wheel_msg);
ros::Publisher right_wheel_pub("rw_speed", &r_wheel_msg);

// Placeholders for wheel speeds in rad/s
double L_WheelVelocity;
double R_WheelVelocity;

// Pololu Motor Driver Initialization
// Rx, Tx, Reset Pins
PololuQik2s12v10 pmd(PMD_RX, PMD_TX, PMD_RESET);

// Placeholders for motor commands
double lm_cmd;
double rm_cmd;

// Placeholders for speed commands
double lw_cmd_spd;
double rw_cmd_spd;

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
               0.75,
               0.0,
               0.15,
               REVERSE);

PID R_DCMotorPID(&R_WheelVelocity,
               &R_PIDout,
               &rw_cmd_spd,
               0.75,
               0.0,
               0.15,
               REVERSE);

// Mode Setter -> 0 for Safe Mode
//             -> 1 for Normal Functionality
int mode = 0;

EmbeddedCollector::EmbeddedCollector(){

  // Initialze ROS communications
  rosInit(nh);

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

// Motor Callback
void EmbeddedCollector::MotorCallback(const scr_proto::SpeedCommand& motor_com){
  lw_cmd_spd = motor_com.left_motor_w;
  rw_cmd_spd = motor_com.right_motor_w;
}

// Command Callback
void EmbeddedCollector::CommandCallback(const std_msgs::Int32& mode_msg){

  if(mode_msg.data == 1){
    mode = mode_msg.data;
    nh.loginfo("Arduino Normal Mode Active");
  }
  else if(mode_msg.data == 0){
    mode = mode_msg.data;
    nh.loginfo("Arduino Safe Mode Active");
  }
  else{
    nh.loginfo("Invalid Mode Command.  Modes are 0 for Safe mode and 1 for normal mode");
  }

}


void EmbeddedCollector::rosInit(ros::NodeHandle_<ArduinoHardware, 5, 5, 256, 256>& nh){

  // Up Baud Rate
  nh.getHardware()->setBaud(115200);

  // Subscriber to speed commands on computer
  ros::Subscriber<scr_proto::SpeedCommand> motor_sub("speed_command", EmbeddedCollector::MotorCallback);
  ros::Subscriber<std_msgs::Int32> command_sub("arduino_mode", EmbeddedCollector::CommandCallback);

  // ROS Stuff
  nh.initNode();
  
  //Command Subscriber
  nh.subscribe(motor_sub);
  nh.subscribe(command_sub);
  
  // Wheel Speeds in rad/s
  nh.advertise(right_wheel_pub);
  nh.advertise(left_wheel_pub);

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

  // Compute New Control Values
  L_DCMotorPID.Compute();
  R_DCMotorPID.Compute();

  lm_cmd = 0;
  rm_cmd = 0;

  // Stop motors directly
  pmd.setSpeeds(0, 0);
}

void EmbeddedCollector::publish(){

  // Populate messages
  l_wheel_msg.data = L_WheelVelocity;
  r_wheel_msg.data = R_WheelVelocity;

  // Publish Data
  left_wheel_pub.publish( &l_wheel_msg );
  right_wheel_pub.publish( &r_wheel_msg );  

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
    // Publish data to ROS
    publish();

    // Handle Callbacks
    nh.spinOnce();
    
    delay(42);
}
