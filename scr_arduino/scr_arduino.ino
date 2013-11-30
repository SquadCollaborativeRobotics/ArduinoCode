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
#include <EmbeddedCollector.h>
#include "ros.h"

EmbeddedCollector::EmbeddedCollector ec;

void setup()
{

}

void loop()
{

  // Object Handles all functionality
  ec.spin();

}

