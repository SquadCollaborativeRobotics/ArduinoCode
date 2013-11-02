/*
 * Squad Collaborative Robotics
 * Prototype Embedded Collector Bot Code
 */


#include <ros.h>
#include <std_msgs/Int32.h>
#include <Event.h>
#include <Timer.h>
#include <Encoder.h>
//#include <PololuQik.h>
//#include <SoftwareSerial.h>

// Encoder stuff
#define LEFT_ENCODER_A_PIN 2
#define LEFT_ENCODER_B_PIN 4
#define RIGHT_ENCODER_A_PIN 3
#define RIGHT_ENCODER_B_PIN 5
#define ENCODER_TIME_DELAY_MS 100
#define TICKS_PER_REVOLUTION 750

// Ros Initialization stuff
ros::NodeHandle  nh;
std_msgs::Int32 l_enc_msg;
std_msgs::Int32 r_enc_msg;

ros::Publisher left_encoder_pub("left_encoder", &l_enc_msg);
ros::Publisher right_encoder_pub("left_encoder", &r_enc_msg);

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
void leftMotorCallback(const std_msgs::Int32& lm_msg){
  lm_cmd = lm_msg.data;
}

void rightMotorCallback(const std_msgs::Int32& rm_msg){
  rm_cmd = rm_msg.data;
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

  // Convert to radians/sec
  L_EncoderVelocity /= (double)TICKS_PER_REVOLUTION;
  R_EncoderVelocity /= (double)TICKS_PER_REVOLUTION;
  
  // Set last known encoder ticks
  L_LastEncoderValue = L_encoderValue;
  R_LastEncoderValue = R_encoderValue;
}

ros::Subscriber<std_msgs::Int32> leftMotorSub("left_motor", leftMotorCallback);
ros::Subscriber<std_msgs::Int32> rightMotorSub("right_motor", rightMotorCallback);


// Rx, Tx, Reset Pins
//PololuQik2s12v10 pmd(2, 3, 4);

void setup()
{
  // ROS Stuff
  nh.initNode();
  nh.subscribe(leftMotorSub);
  nh.subscribe(rightMotorSub);
  nh.advertise(left_encoder_pub);
  nh.advertise(right_encoder_pub);

  // Motor Driver Init
  //pmd.init();
}

void loop()
{
  left_encoder_pub.publish( &l_enc_msg );
  right_encoder_pub.publish( &r_enc_msg );
  nh.spinOnce();
  //pmd.setSpeeds(lm_cmd, rm_cmd);
  delay(5);
}

