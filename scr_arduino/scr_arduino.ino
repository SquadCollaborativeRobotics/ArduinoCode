/*
 * Squad Collaborative Robotics
 * Prototype Embedded Collector Bot Code
   Arduino Mega 2560 R3
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
#define LEFT_ENCODER_A_PIN 2
#define LEFT_ENCODER_B_PIN 4
#define RIGHT_ENCODER_A_PIN 3
#define RIGHT_ENCODER_B_PIN 5
#define ENCODER_TIME_DELAY_MS 100
#define TICKS_PER_REVOLUTION 3200
#define PI 3.141592654

// Pololu Motor Driver Pins
#define PMD_TX 14
#define PMD_RX 15
#define PMD_RESET 8

// Ros Initialization stuff
ros::NodeHandle_<ArduinoHardware, 5, 5, 256, 256>  nh;

// Wheel Speed Messages
std_msgs::Float32 l_wheel_msg;
std_msgs::Float32 r_wheel_msg;

// Motors Command Message
scr_proto::SpeedCommand motor_msg;

// Initialize Publishers
ros::Publisher left_wheel_pub("lw_speed", &l_wheel_msg);
ros::Publisher right_wheel_pub("rw_speed", &r_wheel_msg);

//os::Time last_encoder_time = nh.now();

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

Timer g_TimerEncoder;
float L_EncoderVelocity = 0; // Radians per second
long L_LastEncoderValue = 0;
double L_EncoderDelta = 0; 
double L_LastEncoderDelta = 0; 
float R_EncoderVelocity = 0; // Radians per second
long R_LastEncoderValue = 0;
double R_EncoderDelta = 0; 
double R_LastEncoderDelta = 0; 
double beta = .90;  // Low Pass Parameter for encoder velocity

// Speed Controller Initialization
double L_PIDout = 0;
double R_PIDout = 0;

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

// Motor Callbacks
void MotorCallback(const scr_proto::SpeedCommand& motor_com){
  lw_cmd_spd = motor_com.left_motor_w;
  rw_cmd_spd = motor_com.right_motor_w;
}

void updateEncoderReading() {
  // Get raw ticks
  long L_encoderValue = L_DCMotorEncoder.read();
  long R_encoderValue = R_DCMotorEncoder.read();
  
  // Get position in revolutions (includes multiple revolutions)
// Serial.println(encoderValue);
  L_EncoderDelta = (double)(L_encoderValue - L_LastEncoderValue) * (1-beta) + L_LastEncoderDelta * beta;
  R_EncoderDelta = (double)(R_encoderValue - R_LastEncoderValue) * (1-beta) + R_LastEncoderDelta * beta;
// Serial.println(g_EncoderAngle);
  
  //double dt = nh.now().toSec() - last_encoder_time.toSec();
  double dt = micros() - last_encoder_time;

  // Get velocity in ticks/sec
  L_EncoderVelocity = 1000000*(double)(L_EncoderDelta)/(double)dt;
  R_EncoderVelocity = 1000000*(double)(R_EncoderDelta)/(double)dt;

  // Convert to revolutions/sec
  L_EncoderVelocity /= (double)TICKS_PER_REVOLUTION;
  R_EncoderVelocity /= (double)TICKS_PER_REVOLUTION;
  
  // Convert to rad/s
  L_WheelVelocity = L_EncoderVelocity * 2 * PI;
  R_WheelVelocity = R_EncoderVelocity * 2 * PI;

  // Set last known encoder ticks
  L_LastEncoderValue = L_encoderValue;
  R_LastEncoderValue = R_encoderValue;

  L_LastEncoderDelta = L_EncoderDelta;
  R_LastEncoderDelta = R_EncoderDelta;

  last_encoder_time = micros();
}

ros::Subscriber<scr_proto::SpeedCommand> motor_sub("speed_command", MotorCallback);


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
  // Get Freshest Values
  updateEncoderReading();

  // Compute New Control Values
  L_DCMotorPID.Compute();
  R_DCMotorPID.Compute();

  // Update motor command
  lm_cmd += L_PIDout;
  rm_cmd += R_PIDout;

  // Populate messages
  l_wheel_msg.data = L_WheelVelocity;
  r_wheel_msg.data = R_WheelVelocity;

  // Publish Data
  left_wheel_pub.publish( &l_wheel_msg );
  right_wheel_pub.publish( &r_wheel_msg );
  
  // Handle Callbacks
  nh.spinOnce();
  
  // Write Speeds to motor driver
  pmd.setSpeeds(lm_cmd, rm_cmd);
  
  delay(30);
}

