/*
 * Squad Collaborative Robotics
 * Prototype Embedded Collector Bot Code
 */

#include <ros.h>
#include <std_msgs/Int32.h>
//#include <PololuQik.h>
//#include <SoftwareSerial.h>

// Placeholders for motor commands
int lm_cmd = 0;
int rm_cmd = 0;

// Motor Callbacks
void leftMotorCallback(const std_msgs::Int32& lm_msg){
  lm_cmd = lm_msg.data;
}

void rightMotorCallback(const std_msgs::Int32& rm_msg){
  rm_cmd = rm_msg.data;
}


ros::NodeHandle  nh;
std_msgs::Int32 int_msg;
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
  
  // Motor Driver Init
  //pmd.init();
}

void loop()
{
  int_msg.data = lm_cmd;
  chatter.publish( &int_msg );
  nh.spinOnce();
  //pmd.setSpeeds(lm_cmd, rm_cmd);
  delay(5);
}
