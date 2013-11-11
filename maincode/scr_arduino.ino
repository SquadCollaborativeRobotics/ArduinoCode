/*
 * Squad Collaborative Robotics
 * Prototype Embedded Collector Bot Code
   Arduino Mega 2560 R3
 */


#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
//#include <Event.h>
#include <Timer.h>
#include <Encoder.h>
#include <PololuQik.h>
#include <SoftwareSerial.h>
#include <scr_proto/DiffCommand.h>

// Encoder stuff
#define LEFT_ENCODER_A_PIN 2
#define LEFT_ENCODER_B_PIN 4
#define RIGHT_ENCODER_A_PIN 3
#define RIGHT_ENCODER_B_PIN 5
#define ENCODER_TIME_DELAY_MS 100
#define TICKS_PER_REVOLUTION 750
#define PI 3.141592654

// Pololu Motor Driver Connections
#define PMD_TX 14
#define PMD_RX 15
#define PMD_RESET 8   

// Ros Initialization stuff
ros::NodeHandle_<ArduinoHardware, 5, 5, 256, 256>  nh;

// Encoder Messages
std_msgs::Int32 l_enc_msg;
std_msgs::Int32 r_enc_msg;

// Wheel Speed Messages
std_msgs::Float32 l_wheel_msg;
std_msgs::Float32 r_wheel_msg;

// Motors Command Message
scr_proto::DiffCommand motor_msg;

// Initialize Publishers
//ros::Publisher left_encoder_pub("left_encoder", &l_enc_msg);
ros::Publisher left_wheel_pub("lw_speed", &l_wheel_msg);
//ros::Publisher right_encoder_pub("right_encoder", &l_enc_msg);
ros::Publisher right_wheel_pub("rw_speed", &r_wheel_msg);

// Placeholders for wheel speeds in rad/s
double L_WheelVelocity = 0.0;
double R_WheelVelocity = 0.0;

// Placeholders for motor commands
int lm_cmd = 0;
int rm_cmd = 0;

// Encoder initialization
Encoder L_DCMotorEncoder(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN);
Encoder R_DCMotorEncoder(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN);

Timer g_TimerEncoder;
float L_EncoderVelocity = 0; // Radians per second
long L_LastEncoderValue = 0;
double L_EncoderAngle = 0; // In revolutions (360.0 degrees = 1.0)
float R_EncoderVelocity = 0; // Radians per second
long R_LastEncoderValue = 0;
double R_EncoderAngle = 0; // In revolutions (360.0 degrees = 1.0)

// Motor Callbacks
void MotorCallback(const scr_proto::DiffCommand& motor_com){
  lm_cmd = motor_com.left_motor;
  rm_cmd = motor_com.right_motor;
}

void updateEncoderReading() {
  // Get raw ticks
  long L_encoderValue = L_DCMotorEncoder.read();
  long R_encoderValue = R_DCMotorEncoder.read();
  
  // Get position in revolutions (includes multiple revolutions)
// Serial.println(encoderValue);
  L_EncoderAngle = (double)L_encoderValue / (double)TICKS_PER_REVOLUTION;
  R_EncoderAngle = (double)R_encoderValue / (double)TICKS_PER_REVOLUTION;
// Serial.println(g_EncoderAngle);
  
  // Get velocity in ticks/sec
  L_EncoderVelocity = 1000.0*(float)(L_encoderValue-L_LastEncoderValue)/(float)ENCODER_TIME_DELAY_MS;
  R_EncoderVelocity = 1000.0*(float)(R_encoderValue-L_LastEncoderValue)/(float)ENCODER_TIME_DELAY_MS;

  // Convert to revolutions/sec
  L_EncoderVelocity /= (double)TICKS_PER_REVOLUTION;
  R_EncoderVelocity /= (double)TICKS_PER_REVOLUTION;
  
  // Convert to rad/s
  L_WheelVelocity = L_EncoderVelocity * 2 * PI;
  R_WheelVelocity = R_EncoderVelocity * 2 * PI;

  // Set last known encoder ticks
  L_LastEncoderValue = L_encoderValue;
  R_LastEncoderValue = R_encoderValue;
}

ros::Subscriber<scr_proto::DiffCommand> motor_sub("motor_command", MotorCallback);


// Rx, Tx, Reset Pins
PololuQik2s12v10 pmd(PMD_RX, PMD_TX, 8);

void setup()
{
  // ROS Stuff
  nh.initNode();
  
  //Command Subscriber
  nh.subscribe(motor_sub);
  
  // Encoder Pubs
  //nh.advertise(left_encoder_pub);
  //nh.advertise(right_encoder_pub);
  
  // Wheel Speeds in rad/s
  nh.advertise(right_wheel_pub);
  nh.advertise(left_wheel_pub);
  
  // Motor Driver Init
  pmd.init();
}

void loop()
{
  // Get Freshest Values
  updateEncoderReading();

  // Populate messages
  //l_enc_msg.data = L_LastEncoderValue;
  //r_enc_msg.data = R_LastEncoderValue;
  l_wheel_msg.data = L_WheelVelocity;
  r_wheel_msg.data = R_WheelVelocity;

  // Publish Data
  //left_encoder_pub.publish( &l_enc_msg );
  //right_encoder_pub.publish( &r_enc_msg );
  left_wheel_pub.publish( &l_wheel_msg );
  right_wheel_pub.publish( &r_wheel_msg );
  
  // Handle Callbacks
  nh.spinOnce();
  
  // Write Speeds to motor driver
  pmd.setSpeeds(lm_cmd, rm_cmd);
  
  delay(10);
}

