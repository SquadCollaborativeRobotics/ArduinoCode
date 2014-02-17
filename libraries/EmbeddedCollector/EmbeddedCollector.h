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

    /*
     *  Function spin: Responsible for functionality of embedded code
     *  First turns through the PID motor control Loop   
     *  Then aggregates the Encoder Data and sends to main thread to publish over ROS
     */
    void spin();

    /*
     *  Function init: Called in Arduino setup.  Responsible for
     *  initializing all the systems embedded layer depends on.
     */
    void initialize();

    /*
     *  Function getWheelVels: Called from Arduino Wrapper to get the velocities of the wheels
     *  @ Return -> out[0] : Left Wheel Velocity in rad/s
     *              out[1] : Right Wheel Velocity in rad/s
     */
    float* getWheelVels();

    /*
     *  Function setMode: Sets the mode of the hardware
     *  @ Param -> mode : mode of the robot 0 for safe, 1 for active. 
     */
    void setMode(int mode);

    /*
     *  Function setMotorCommands: Sets the motor commanded speeds
     *  @ Params -> lw_cmd : Commanded left wheel speed in rad/s
     *              rw_cmd : Commanded right wheel speed in rad/s
     */
    void setMotorCommands(float lw_cmd, float rw_cmd);
	
  
  private:
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
     *  Function safeControlLoop: Responsible for keeping base still and PID up to date
     *  Sets commanded speeds to zero, computes PID output, set's output to zero, then
     *  Sets the stops the motors.
     */
    void safeControlLoop();

};

#endif
