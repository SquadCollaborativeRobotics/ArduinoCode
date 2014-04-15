/*
 * Squad Collaborative Robotics
 * Prototype Embedded Collector Bot Code
 * Arduino Mega 2560 R3
 */


// ROS Includes
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <scr_proto/SpeedCommand.h>

// Speed Controller Includes
#include <Timer.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <PololuQik.h>
#include <SoftwareSerial.h>

// Encoder Pins and Variables
#define LEFT_ENCODER_A_PIN 3
#define LEFT_ENCODER_B_PIN 5
#define RIGHT_ENCODER_A_PIN 2
#define RIGHT_ENCODER_B_PIN 4
#define TICKS_PER_REVOLUTION 3200
#define PI 3.141592654

// Pololu Motor Driver Pins
#define PMD_TX 14
#define PMD_RX 15
#define PMD_RESET 8

// Loop Publish Rate
#define PUB_RATE 33

// Last millis() publish time
unsigned long last_pub_time = 0;

// Ros Initialization stuff
ros::NodeHandle_<ArduinoHardware, 3, 3, 128, 128>  nh;

// Wheel Speed Messages
std_msgs::Float32 l_wheel_msg;
std_msgs::Float32 r_wheel_msg;

// Motors Command Message
scr_proto::SpeedCommand motor_msg;
std_msgs::Int32 mode_msg;

// Initialize Publishers
ros::Publisher left_wheel_pub("lw_speed", &l_wheel_msg);
ros::Publisher right_wheel_pub("rw_speed", &r_wheel_msg);

double last_encoder_time = micros();

// Placeholders for wheel speeds in rad/s
double L_WheelVelocity = 0.0;
double R_WheelVelocity = 0.0;

// Placeholders for motor commands
double lm_cmd = 0;
double rm_cmd = 0;

// Placeholders for speed commands
double lw_cmd_spd = 0;
double rw_cmd_spd = 0;

// Encoder initialization
Encoder L_DCMotorEncoder(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN);
Encoder R_DCMotorEncoder(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN);

float L_EncoderVelocity = 0; // Radians per second
long L_LastEncoderValue = 0;
double L_EncoderDelta = 0; 
double L_LastEncoderDelta = 0; 
float R_EncoderVelocity = 0; // Radians per second
long R_LastEncoderValue = 0;
double R_EncoderDelta = 0; 
double R_LastEncoderDelta = 0; 
double beta = .50;  // Low Pass Parameter for encoder velocity

// Speed Controller Initialization
double L_PIDout = 0;
double R_PIDout = 0;

// Input, Output, Setpoint, P, I, D, DIRECT/REVERSE)
PID L_DCMotorPID(&L_WheelVelocity,
                 &L_PIDout,
                 &lw_cmd_spd,
                 0.950,
                 0.000,
                 0.0720,
                 REVERSE);

PID R_DCMotorPID(&R_WheelVelocity,
                 &R_PIDout,
                 &rw_cmd_spd,
                 0.950,
                 0.000,
                 0.0720,
                 REVERSE);

int mode = 0;

// Motor Callbacks
void MotorCallback(const scr_proto::SpeedCommand& motor_com){
  lw_cmd_spd = motor_com.left_motor_w;
  rw_cmd_spd = motor_com.right_motor_w;
}

// Command Callback
void CommandCallback(const std_msgs::Int32& mode_msg){

  if(mode_msg.data == 1){
    mode = mode_msg.data;
    nh.loginfo("Arduino Normal Mode Active");
  }
  else if(mode_msg.data == 0){
    mode = mode_msg.data;
    nh.loginfo("Arduino Safe Mode Active");
  }
  else{
    nh.loginfo("Invalid Mode Command. Modes are 0 for Safe mode and 1 for normal mode");
  }

}

void updateEncoderReading() {
  // Get raw ticks
  long L_encoderValue = L_DCMotorEncoder.read();
  long R_encoderValue = R_DCMotorEncoder.read();
  
  // Current time in micro seconds
  long now = micros();
  
  long dt = (now - last_encoder_time);
  
  // Get position in ticks with low pass filter
  L_EncoderDelta = (double)(L_encoderValue - L_LastEncoderValue) * (1-beta) + L_LastEncoderDelta * beta;
  R_EncoderDelta = (double)(R_encoderValue - R_LastEncoderValue) * (1-beta) + R_LastEncoderDelta * beta;
  
  // Get velocity in revs/sec
  L_EncoderVelocity = 1000000.0/dt * L_EncoderDelta/TICKS_PER_REVOLUTION;
  R_EncoderVelocity = 1000000.0/dt * R_EncoderDelta/TICKS_PER_REVOLUTION;
  
  // Convert to rad/sec
  L_WheelVelocity = L_EncoderVelocity * 2 * PI;
  R_WheelVelocity = R_EncoderVelocity * 2 * PI;

  // Update previous values
  L_LastEncoderValue = L_encoderValue;
  R_LastEncoderValue = R_encoderValue;

  last_encoder_time = now;

  L_LastEncoderDelta = L_EncoderDelta;
  R_LastEncoderDelta = R_EncoderDelta;

}

ros::Subscriber<scr_proto::SpeedCommand> motor_sub("speed_command", MotorCallback);
ros::Subscriber<std_msgs::Int32> command_sub("arduino_mode", CommandCallback);

// Rx, Tx, Reset Pins
PololuQik2s12v10 pmd(PMD_RX, PMD_TX, PMD_RESET);

void setup()
{

  // Up Baud Rate
  nh.getHardware()->setBaud(115200);

  // ROS Stuff
  nh.initNode();
  
  //Command Subscriber
  nh.subscribe(motor_sub);
  nh.subscribe(command_sub);
  
  // Wheel Speeds in rad/s
  nh.advertise(right_wheel_pub);
  nh.advertise(left_wheel_pub);

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

void loop()
{
  if (millis() - last_pub_time > PUB_RATE)
  {
    last_pub_time = millis();

    if(mode == 0){
      lw_cmd_spd = 0;
      rw_cmd_spd = 0;
    }
    // Get Freshest Values
    updateEncoderReading();

    // Compute New Control Values
    L_DCMotorPID.Compute();
    R_DCMotorPID.Compute();

    // Update motor command
    lm_cmd += L_PIDout;
    rm_cmd += R_PIDout;

   if(mode == 0){
      lm_cmd = 0;
      rm_cmd = 0;
    }
    // Populate messages
    l_wheel_msg.data = L_WheelVelocity;
    r_wheel_msg.data = R_WheelVelocity;

    // Publish Data
    left_wheel_pub.publish( &l_wheel_msg );
    right_wheel_pub.publish( &r_wheel_msg );
    
    // Write Speeds to motor driver
    pmd.setSpeeds(lm_cmd, rm_cmd);
  }

  // Handle Callbacks
  nh.spinOnce();
}
