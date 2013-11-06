/*
 * rosserial Temperature Sensor Example modified for ultrasonic
 * 
 * This tutorial demonstrates the usage of the
 * Sparkfun TMP102 Digital Temperature Breakout board
 * http://www.sparkfun.com/products/9418
 * 
 * Source Code Based off of:
 * http://wiring.org.co/learning/libraries/tmp102sparkfun.html
 */

#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

std_msgs::Float32 dist_msg;
ros::Publisher pub_temp("ultrasonic_dist", &dist_msg);

#define trigPin 13
#define echoPin 12


void setup()
{
  nh.initNode();
  nh.advertise(pub_temp);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop()
{
  
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  
  if (distance >= 200 || distance <= 0){
      dist_msg.data = -1;
      pub_temp.publish(&dist_msg);
  }
  else {

      dist_msg.data = distance * 0.01;
      pub_temp.publish(&dist_msg);
  }
  nh.spinOnce();
}
