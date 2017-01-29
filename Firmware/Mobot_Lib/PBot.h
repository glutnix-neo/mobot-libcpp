// ************************************************************************
// Filename: 	PBot.h
// Description:	Header File for PBot.cpp
// Revision: 	v01.01.01
// Author:		Efren S. Cruzat II
//				(PhilRobotics.com, glutnix_neo of electronicslab.ph/forum)
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
// v00.00.02      20101119     Efren C.         Intiial Library Release
// v00.01.00      20101126     Efren C.         Enhancements:
//                                              				- added KI_OBJECT_BOTHSIDES on obstacle states
//                                      				- defined P-Bot Modes
// v01.00.00      20101216     Efren C.         Enhancements:
//                                              				- Added Telemetry Functions
//                                              				- Separate Low Level Mobot files
//                                                  			 for easy configuration and porting
// v01.01.00      20110204     Efren C.         Enhancements:
//                                              				- Added D-Protocol Functions
//                                              				- Added methods for reporting wall and
//                                                  			 line sensors
// v00.01.01       20120118     Efren C.        	Enhancements:
//								 - removed included files
//									WProgram.h
//									WConstants.h
//									Telemetry.h
// ************************************************************************
#ifndef PBot_h
#define PBot_h

//#include <WProgram.h>
//#include <WConstants.h>
#include <LowLevelMobot.h>
//#include <Telemetry.h>
#include <DProtocol.h>

// *** User Friendly Keywords ***
// P-Bot Modes
#define SUMO				   0
#define LINETRACKING		   1
#define MICROMOUSE			   2
#define REMOTECONTROL          3
#define WHEELBALANCE		   4
#define VACUUMCLEANER          5

// Line/Wall Sensor Polarity
#define WHITE                  KBL_WHITE
#define BLACK                  KBL_BLACK

// Motor Predefined Speeds
#define FULLSPEED              KB_FULLSPEED
#define HIGHSPEED              KB_HIGHSPEED
#define MIDSPEED               KB_MIDSPEED
#define LOWSPEED               KB_LOWSPEED

// Motor Spin Direction
#define FORWARD                KBL_FORWARD_DIR
#define REVERSE                KBL_REVERSE_DIR

// Obstacle Input States
#define NO_OBSTACLE            KI_NO_OBSTACLE
#define OBJECT_LEFT            KI_OBJECT_LEFT
#define OBJECT_LEFTCENTER      KI_OBJECT_LEFTCENTER
#define OBJECT_RIGHT           KI_OBJECT_RIGHT
#define OBJECT_RIGHTCENTER     KI_OBJECT_RIGHTCENTER
#define OBJECT_FARFRONT        KI_OBJECT_FARFRONT
#define OBJECT_NEARFRONT       KI_OBJECT_NEARFRONT
#define OBJECT_BOTHSIDES	   KI_OBJECT_BOTHSIDES

// Line Input States
#define NO_LINE                KI_NO_LINE
#define LINE_LEFT              KI_LINE_LEFT
#define LINE_LEFTCENTER        KI_LINE_LEFTCENTER
#define LINE_RIGHT             KI_LINE_RIGHT
#define LINE_RIGHTCENTER       KI_LINE_RIGHTCENTER
#define OVER_LINE              KI_OVER_LINE
#define LINE_CENTER            KI_LINE_CENTER
#define LINE_REAR              KI_LINE_CENTER
#define LINE_FRONT             KI_LINE_FRONT

class PBotClass : public LowLevelMobotClass
{
  private:
    // *** private variables **********************************************
    char strDProtID[4];

    // *** private functions **********************************************
    #if !CONFIG_DISABLE_TELEMETRY
    PBotTelemetryClass PBotTelemetry;
    #endif

    DProtocolClass PBotControl;

    void startDProtocol(boolean blEnableRemoteControl, boolean blEnableTelemetry, int iDBaudRate);

  public:
    // *** public variables ***********************************************
    // Working Buffer
    char strDProtocolID[4];
    char strCommandCode[5];
    char strParam1[7];
    char strParam2[7];

    int iObstacleLocation;
    int iLineLocation;

    //boolean blDPacketAvailable;
    boolean blBufferOverflow;
    boolean blTerminationError;

    boolean blObstacleSensorPolarity;                           // 0 detect white, 1 detect white
    boolean blLineSensorPolarity;                               // 0 detect white, 1 detect white

    // *** public functions ***********************************************
	PBotClass(void);

    // High Level PBot Motor Control Functions
    void begin(int iMode, boolean blObstaclePolarity, boolean blLinePolarity);
                                                                // mode 0: Sumo Robot
                                                                //      1: Line Tracking Robot
                                                                //      2: Micromouse Robot         (Future Option)
                                                                //      3: Remote Controlled Car    (Future Option)
                                                                //      4: 2 Wheel Balancing Robot  (Future Option)
                                                                //      5: Vacuum Cleaner           (Future Upgrade)
                                                                // polarity can be 0 - White Detection , 1 - Black Detection

    void begin(int iMode, boolean blObstaclePolarity, boolean blLinePolarity, boolean blEnableTelemetry, int iDBaudRate);
    void moveForward(byte bSpeed);                       // bSpeed is from 0 - 255
    void moveBackward(byte bSpeed);                      // bSpeed is from 0 - 255
    void turnLeft(byte bSpeed);                          // bSpeed is from 0 - 255
    void turnRight(byte bSpeed);                         // bSpeed is from 0 - 255
    void rotateLeft(byte bSpeed);                        // bSpeed is from 0 - 255
    void rotateRight(byte bSpeed);                       // bSpeed is from 0 - 255
    void stopMotors(void);

    // Object / Wall Sensor Detection
    int detectObstacle(void);

    // Line Sensor Detection
    int detectLine(void);

    // Telemetry
    #if !CONFIG_DISABLE_TELEMETRY
    void pollSensors(unsigned long ulPollingInterval);
    void reportWallSensors(void);
    void reportLineSensors(void);
    #endif

    // Remote Control
    void parseIncomingBytes(void);
    void interpretDProtocolCommands(void);
    void resetDProtocol(void);
};
#endif
// *********************************************************************
// End of PBot.h
// *********************************************************************
