ArduinoCode
===========

All of the Arduino code (and libraries)


To install rosserial:

Follow the instructions here. DO NOT USE sudo apt-get install rosserial stuff if you're running groovy... Follow the instructions for installing from source into the src directory of the catkin workspace

http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

Make sure you install the CORRECT (groovy) arduino libraries into the sketchbook libraries folder.  I have setup the arduino sketchbook directory to be here

Open the arduino preferences file and change the location of the sketchbook to the correct directories sketchbook (<something>/ArduinoCode/sketchbook)

Currently can't get motor driver code on Uno to work with rosserial.  I think this has to do with the whole one serial port thing.  We'll need to retest when we get our Mega in.
