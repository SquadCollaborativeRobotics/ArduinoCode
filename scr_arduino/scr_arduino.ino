/*
 * Squad Collaborative Robotics
 * Prototype Embedded Collector Bot Code
 * Arduino Mega 2560 R3
 */

// Speed Controller Includes
#include "Timer.h"
#include "Encoder.h"
#include "PID_v1.h"
#include "PololuQik.h"
#include "SoftwareSerial.h"
#include "EmbeddedCollector.h"

// ROS Includes
#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
//#include <scr_proto/SpeedCommand.h>

ros::NodeHandle_<ArduinoHardware, 10, 10, 512, 512> nh;
EmbeddedCollector::EmbeddedCollector ec;

// Placeholders for speed commands
double l_cmd_spd;
double r_cmd_spd;

// Wheel Speed Messages
std_msgs::Float32 l_wheel_msg;
std_msgs::Float32 r_wheel_msg;

// Initialize Publishers
ros::Publisher left_wheel_pub("lw_speed", &l_wheel_msg);
ros::Publisher right_wheel_pub("rw_speed", &r_wheel_msg);



void publish(float L_WheelVelocity, float R_WheelVelocity){

  // Populate messages
  l_wheel_msg.data = L_WheelVelocity;
  r_wheel_msg.data = R_WheelVelocity;

  // Publish Data
  left_wheel_pub.publish( &l_wheel_msg );
  right_wheel_pub.publish( &r_wheel_msg );  

}


void spin(){

	ec.spin();
	float* vels = ec.getWheelVels();
	
	// Publish data to ROS
  publish(vels[0], vels[1]);

  // Handle Callbacks
  nh.spinOnce();
    
  delay(10);

}



// Motor Callback
void motorCallback(const scr_proto::SpeedCommand& motor_com){
  l_cmd_spd = motor_com.left_motor_w;
  r_cmd_spd = motor_com.right_motor_w;

  ec.setMotorCommands(l_cmd_spd, r_cmd_spd);
}

ros::Subscriber<scr_proto::SpeedCommand> motor_sub("speed_command", motorCallback);

// Command Callback
void commandCallback(const std_msgs::Int32& mode_msg){

  if(mode_msg.data == 1){
    ec.setMode(mode_msg.data);
    nh.loginfo("Arduino Normal Mode Active");
  }
  else if(mode_msg.data == 0){
    ec.setMode(mode_msg.data);
    nh.loginfo("Arduino Safe Mode Active");
  }
  else{
    nh.loginfo("Invalid Mode Command.  Modes are 0 for Safe mode and 1 for normal mode");
  }

}

// Subscriber to speed commands on computer

ros::Subscriber<std_msgs::Int32> command_sub("arduino_mode", commandCallback);

void setup()
{

  nh.initNode();
  nh.getHardware()->setBaud(115200);
  //nh.loginfo("baud rate set");\

  // ROS Stuff
  
  //Command Subscriber
  nh.subscribe(motor_sub);
  nh.subscribe(command_sub);
  
  // Wheel Speeds in rad/s
  nh.advertise(right_wheel_pub);
  nh.advertise(left_wheel_pub);

}

void loop()
{

  // Object Handles all functionality
  spin();

  //nh.loginfo("Im Alive");

}

