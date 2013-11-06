/*
 * rosserial Temperature Sensor Example
 * 
 * This tutorial demonstrates the usage of the
 * Sparkfun TMP102 Digital Temperature Breakout board
 * http://www.sparkfun.com/products/9418
 * 
 * Source Code Based off of:
 * http://wiring.org.co/learning/libraries/tmp102sparkfun.html
 */

#include <NewPing.h>
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;


std_msgs::Float32 dist_msg;
ros::Publisher pub_temp("ultrasonic", &dist_msg);

#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup()
{
  nh.initNode();
  nh.advertise(pub_temp);
}

void loop()
{

  delay(50);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).

  float distance = uS / US_ROUNDTRIP_CM;

  dist_msg.data = distance * 0.01;
  pub_temp.publish(&dist_msg);

  nh.spinOnce();
}
