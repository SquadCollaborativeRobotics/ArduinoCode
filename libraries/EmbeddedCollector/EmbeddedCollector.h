/*
 * Squad Collaborative Robotics
 * Prototype Embedded Collector Bot Code
 * Arduino Mega 2560 R3
 */

#ifndef EmbeddedCollector_h
#define EmbeddedCollector_h

// ROS Includes
#include "ros.h"
#include "Arduino.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "scr_proto/SpeedCommand.h"

#include "Timer.h"
#include "Encoder.h"
#include "PID_v1.h"
#include "PololuQik.h"
#include "SoftwareSerial.h"

// Encoder Pins and Variables
#define LEFT_ENCODER_A_PIN 2
#define LEFT_ENCODER_B_PIN 4
#define RIGHT_ENCODER_A_PIN 3
#define RIGHT_ENCODER_B_PIN 5
#define TICKS_PER_REVOLUTION 3200
#define PI 3.141592654

// Pololu Motor Driver Pins
#define PMD_TX 14
#define PMD_RX 15
#define PMD_RESET 8


class EmbeddedCollector
{

  public:

    EmbeddedCollector();
    /*
     *  Function spin: Responsible for functionality of embedded code
     *  First turns through the PID motor control Loop   
     *  Then aggregates the Encoder Data and publishes over ROS
     *  Finally spins to handle any callbacks
     */
    void spin();	
  
  private:
      /*
       *  Function rosInit: Responsible for ROS relationships
       *  Sets Baud Rate
       *  Instantiates Subscriber
       *  Initializes Node
       *  Exchanges with computer to establish communications
       */
	  void rosInit(ros::NodeHandle_<ArduinoHardware, 5, 5, 256, 256>&);
	  
      /*
       *  Function initMotors: Responsible for starting motor control objects
       *  Initializes variables for control PID controller and motor commands
       *  Sets the PID objects for our purposes
       *  Initializes the Pololu Motor Driver 
       */
	  void initMotors();

	  /*
	   *  Function initEncoders: Responsible for starting Encoder values/objects
	   *  Initializes all parameters needed for use of the Encoders
	   */
	  void initEncoders();

	  /*
	   *  Function MotorCallback:  Responsible for pulling commands into namespace
	   *  Function that is called when a new command speed is sent from the computer
	   */
	  static void MotorCallback(const scr_proto::SpeedCommand&);
	 
      /*
       *  Function updateEncoderReading: Responsible for tracking wheel speeds and encoder values
       *  Pulls in most recent encoder values
       *  Calculates wheel speeds and converts to rad/s
       *  Stores all relevant values in class variables
       */
	  void updateEncoderReading();

	  /*
	   *  Function controlLoop: Responsible for running motor control loop
	   *  Updates Wheel Speeds
	   *  Calculates PID Outputs
	   *  Assings output to Pololu Motor Driver
	   */
	  void controlLoop();

	  /*
	   *  Function publish: Responsible for communicating wheel speeds to ROS computer
	   *  Populates wheel speed messages and uses nodehandle to push them over ROS
	   */
	  void publish();
	
};

#endif
