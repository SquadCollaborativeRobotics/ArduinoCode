/*
 * ros
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <scr_proto/DiffCommand.h>

// Placeholders for
int lm_cmd = 0;
int rm_cmd = 0;

void (const std_msgs::Int32& diff_msg){
  lm = diff_msg.data;
}


ros::NodeHandle  nh;

std_msgs::Int32 int_msg;
ros::Publisher chatter("chatter", &int_msg);
ros::Subscriber<std_msgs::Int32> DiffSub("motor_com", diffCb);



void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(DiffSub);
}

void loop()
{
  int_msg.data = lm;
  chatter.publish( &int_msg );
  nh.spinOnce();
  delay(10);
}
