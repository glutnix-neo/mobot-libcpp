// ************************************************************************
// Filename: 	LowLevelMobot.h
// Description:	Low Level Mobot methods and definitions
// Revision: 	v00.01.01
// Author:	    Efren S. Cruzat II
//		        (PhilRobotics.com, glutnix_neo of electronicslab.ph/forum)
// Compiler:	Arduino v0021
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>
// ************************************************************************
// FW Version      Date        Author           Description
// v00.01.00      20101216     Efren C.        	 Intiial Library Release
// v00.01.01       20120116     Efren C.        	Bug Fixes:
//								- Modified to use Arduino.h instead of WProgram
//							 	 for compatibility with Arduino v1.0
// ************************************************************************
#ifndef LowLevelMobot_h
#define LowLevelMobot_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "WConstants.h"
#endif

// Gizduino Pin    Signal Name
// 2            Collision Sensor 3         (Right)
// 3            Collision Sensor 2         (Center)
// 4            Collision Sensor 1         (Left)
#define KI_RIGHT_WALLSENSOR        2
#define KI_CENTER_WALLSENSOR       3
#define KI_LEFT_WALLSENSOR         4

// 7            Line Sensor 1               (Right)
// 6            Line Sensor 2               (Center)
// 5            Line Sensor 3               (Left)
#define KI_RIGHT_LINESENSOR        5
#define KI_CENTER_LINESENSOR       6
#define KI_LEFT_LINESENSOR         7

// 8            Motor 1 Direction Control   (Right Motor)
// 9            Motor 1 Run Control         (Right Motor)
// 10           Motor 2 Run Control         (Left Motor)
// 11           Motor 2 Direction Control   (Left Motor)
#define KI_RIGHTMOTOR_DIRCTRL      8
#define KI_RIGHTMOTOR_RUNCTRL      9
#define KI_LEFTMOTOR_RUNCTRL      10
#define KI_LEFTMOTOR_DIRCTRL      11

// 12           Right Wheel Tachometer
// 13           Left Wheel Tachometer
#define KI_RIGHTWHEEL_TACHO       12
#define KI_LEFTWHEEL_TACHO        13

// Line/Wall Sensor Polarity
#define KBL_WHITE               false
#define KBL_BLACK               true

// Predefined Motor Speeds
#define KB_MAXSPEED              255

#define KB_FULLSPEED             255
#define KB_HIGHSPEED             200
#define KB_MIDSPEED              150
#define KB_LOWSPEED               75

#define KB_OFFSPEED                0

// Motor Direction Control
#define KBL_FORWARD_DIR          LOW
#define KBL_REVERSE_DIR         HIGH

// Obstacle Input States
enum { KI_NO_OBSTACLE, KI_OBJECT_LEFT, KI_OBJECT_LEFTCENTER, KI_OBJECT_RIGHT, KI_OBJECT_RIGHTCENTER, KI_OBJECT_FARFRONT, KI_OBJECT_NEARFRONT, KI_OBJECT_BOTHSIDES };
// Line Input States
enum { KI_NO_LINE, KI_LINE_LEFT, KI_LINE_LEFTCENTER, KI_LINE_RIGHT, KI_LINE_RIGHTCENTER, KI_OVER_LINE, KI_LINE_CENTER, KI_LINE_FRONT };

// Constants
#define KUL_TACHOPOLLTIMEOUT      3000      // no motion 3secs

// Library Configutation
#define CONFIG_DISABLE_TELEMETRY            0
#define CONFIG_DISABLE_REMOTE_CONTROL       0

class LowLevelMobotClass
{
  private:
    // *** private variables **********************************************

    // *** private functions **********************************************
    boolean readSensorOutput(int iSensorPin, boolean blPolarity);

  public:
    // *** public variables ***********************************************
    boolean blWallDetectedonLeft;
    boolean blWallDetectedonCenter;
    boolean blWallDetectedonRight;

    boolean blLineDetectedonLeft;
    boolean blLineDetectedonCenter;
    boolean blLineDetectedonRight;

    // *** public functions ***********************************************
    LowLevelMobotClass(void);

    // Low Level Motor Control Functions
    void runRightMotor(byte bSpeed, boolean blDirection);    // bSpeed is from 0 - 255
                                                             // blDirection can be forward-0, reverse-1
    void runLeftMotor(byte bSpeed, boolean blDirection);     // bSpeed is from 0 - 255
                                                             // blDirection can be forward-0, reverse-1
    void stopRightMotor(void);
    void stopLeftMotor(void);

    // Low Level Input Polling
    unsigned long  getRightTachoPeriod(boolean blActiveEdge);
    unsigned long  getLeftTachoPeriod(boolean blActiveEdge);

    void pollWallSensors(boolean *blObstacleSensorPolarity);
    void pollLineSensors(boolean *blLineSensorPolarity);
};

#endif
// *********************************************************************
// End of LowLevelMobot.h
// *********************************************************************
