// ************************************************************************
// Filename: 	PBot.cpp
// Description:	Standard Arduino Library for controlling e-gizmo's
//				PBot Mobile Robot Kit (and compatible mobots)
//				visit "http://www.e-gizmo.com/KIT/P-BOT.htm" for more info.
// Revision: 	v01.01.01
// Author:		Efren S. Cruzat II (efren.cruzat (at) PhilRobotics.com)
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
//
// ************************************************************************
// FW Version      Date        Author           Description
// v00.00.02      20101119     Efren C.         Intiial Library Release
//
// v00.01.00      20101126     Efren C.         Bug Fixes:
//                                              				- Obstacle sensor polarity inverted
//                                              			Enhancements:
//								- Added detection of wall on sides,
//								useful for micromouse mode
//
// v01.00.00      20101216     Efren C.         Enhancements:
//                                              				- Added Telemetry Functions
//                                              				- Separate Low Level Mobot files
//                                                  			for easy configuration and porting
//
// v01.01.00      20110204     Efren C.         Enhancements:
//                                              				- Added methods for reporting wall and
//                                                  			line sensors
// v00.01.01       20120118     Efren C.        	Enhancements:
//								 - removed included files
//									WProgram.h
//									WConstants.h
// ************************************************************************
//#include <WProgram.h>
//#include <WConstants.h>
#include "PBot.h"

// ************************************************************************
// *** Start of PBotClass *************************************************
// ************************************************************************

// *** Constructor ********************************************************
PBotClass::PBotClass()
{
    // null
}

// PBot Begin with D-Protocol
void PBotClass::begin(int iMode, boolean blObstaclePolarity, boolean blLinePolarity, boolean blEnableTelemetry, int iDBaudRate)
{
    boolean blEnableRemoteControl = false;

    if(iMode == REMOTECONTROL)
    {
        blEnableRemoteControl = true;
    }

    begin(iMode, blObstaclePolarity, blLinePolarity);
    startDProtocol(blEnableRemoteControl, blEnableTelemetry, iDBaudRate);
}

// Basic PBot Begin
void PBotClass::begin(int iMode, boolean blObstaclePolarity, boolean blLinePolarity)
{
  blObstacleSensorPolarity = !blObstaclePolarity;
  blLineSensorPolarity = blLinePolarity;

  // Initilize Collision Sensor Inputs
  pinMode(KI_RIGHT_WALLSENSOR, INPUT);
  pinMode(KI_CENTER_WALLSENSOR, INPUT);
  pinMode(KI_LEFT_WALLSENSOR, INPUT);

  // Initilize Line Sensor Inputs
  pinMode(KI_RIGHT_LINESENSOR, INPUT);
  pinMode(KI_CENTER_LINESENSOR, INPUT);
  pinMode(KI_LEFT_LINESENSOR, INPUT);

  // Initilize Motor Control Outputs and Initially Low
  digitalWrite(KI_RIGHTMOTOR_DIRCTRL, LOW);
  digitalWrite(KI_RIGHTMOTOR_RUNCTRL, LOW);
  digitalWrite(KI_LEFTMOTOR_RUNCTRL, LOW);
  digitalWrite(KI_LEFTMOTOR_DIRCTRL, LOW);

  pinMode(KI_RIGHTMOTOR_DIRCTRL, OUTPUT);
  pinMode(KI_RIGHTMOTOR_RUNCTRL, OUTPUT);
  pinMode(KI_LEFTMOTOR_RUNCTRL, OUTPUT);
  pinMode(KI_LEFTMOTOR_DIRCTRL, OUTPUT);
}

// *** High Level PBot Control Functions **********************************
void PBotClass::moveForward(byte bSpeed)
{
  runRightMotor(bSpeed, KBL_FORWARD_DIR);
  runLeftMotor(bSpeed, KBL_FORWARD_DIR);
}

void PBotClass::moveBackward(byte bSpeed)
{
  runRightMotor(bSpeed, KBL_REVERSE_DIR);
  runLeftMotor(bSpeed, KBL_REVERSE_DIR);
}

void PBotClass::turnLeft(byte bSpeed)
{
  runRightMotor(KB_FULLSPEED, KBL_FORWARD_DIR);
  runLeftMotor(KB_FULLSPEED-bSpeed, KBL_FORWARD_DIR);
}

void PBotClass::turnRight(byte bSpeed)
{
  runRightMotor(KB_FULLSPEED-bSpeed, KBL_FORWARD_DIR);
  runLeftMotor(KB_FULLSPEED, KBL_FORWARD_DIR);
}

void PBotClass::rotateLeft(byte bSpeed)
{
  runRightMotor(bSpeed, KBL_FORWARD_DIR);
  runLeftMotor(bSpeed, KBL_REVERSE_DIR);
}

void PBotClass::rotateRight(byte bSpeed)
{
  runRightMotor(bSpeed, KBL_REVERSE_DIR);
  runLeftMotor(bSpeed, KBL_FORWARD_DIR);
}

void PBotClass::stopMotors(void)
{
  stopRightMotor();
  stopLeftMotor();
}

// *** Object / Wall Sensor Detection *************************************
int PBotClass::detectObstacle(void)
{
  pollWallSensors(&blObstacleSensorPolarity);

  // Front Near
  if(blWallDetectedonLeft && blWallDetectedonCenter && blWallDetectedonRight)
  {
    iObstacleLocation = KI_OBJECT_NEARFRONT;
  }
  // Left-Center
  else if(blWallDetectedonLeft && blWallDetectedonCenter)
  {
    iObstacleLocation = KI_OBJECT_LEFTCENTER;
  }
  // Right-Center
  else if(blWallDetectedonCenter && blWallDetectedonRight)
  {
    iObstacleLocation = KI_OBJECT_RIGHTCENTER;
  }
  // Both Sides
  else if(blWallDetectedonLeft && blWallDetectedonRight)
  {
  	iObstacleLocation = KI_OBJECT_BOTHSIDES;
  }
  // Front Far
  else if(blWallDetectedonCenter)
  {
    iObstacleLocation = KI_OBJECT_FARFRONT;
  }
  // Leftmost
  else if(blWallDetectedonLeft)
  {
    iObstacleLocation = KI_OBJECT_LEFT;
  }
  // Rightmost
  else if(blWallDetectedonRight)
  {
    iObstacleLocation = KI_OBJECT_RIGHT;
  }
  // No Obstacle
  else
  {
    iObstacleLocation = KI_NO_OBSTACLE;
  }

  return  iObstacleLocation;
}

// *** Line Sensor Detection **********************************************
int PBotClass::detectLine(void)
{
  pollLineSensors(&blLineSensorPolarity);

  // Center
  if(blLineDetectedonLeft && blLineDetectedonCenter && blLineDetectedonRight)
  {
    iLineLocation = KI_OVER_LINE;
  }
  // Left-Center
  else if(blLineDetectedonLeft && blLineDetectedonCenter)
  {
    iLineLocation = KI_LINE_LEFTCENTER;
  }
  // Right-Center
  else if(blLineDetectedonCenter && blLineDetectedonRight)
  {
    iLineLocation = KI_LINE_RIGHTCENTER;
  }
  // Leftmost
  else if(blLineDetectedonLeft)
  {
    iLineLocation = KI_LINE_LEFT;
  }
  // Rightmost
  else if(blLineDetectedonRight)
  {
    iLineLocation = KI_LINE_RIGHT;
  }
  // Center
  else if(blLineDetectedonCenter)
  {
    iLineLocation = KI_LINE_CENTER;
  }
  // No Obstacle
  else
  {
    iLineLocation = KI_NO_LINE;
  }

  return  iLineLocation;
}

// *** Telemetry Methods **************************************************
void PBotClass::startDProtocol(boolean blEnableRemoteControl, boolean blEnableTelemetry, int iDBaudRate)
{
    if(blEnableRemoteControl)
    {
        PBotControl.begin(iDBaudRate, strDProtID, this);
    }

    #if !CONFIG_DISABLE_TELEMETRY
    else if(blEnableTelemetry)
    {
        PBotTelemetry.begin(iDBaudRate, strDProtID, this);
    }
    #endif
}

#if !CONFIG_DISABLE_TELEMETRY
void PBotClass::pollSensors(unsigned long ulPollingInterval)
{
    PBotTelemetry.pollSensors(ulPollingInterval);
}

void PBotClass::reportWallSensors(void)
{
    PBotTelemetry.reportWallSensors(iObstacleLocation);
}

void PBotClass::reportLineSensors(void)
{
    PBotTelemetry.reportLineSensors(iLineLocation);
}
#endif

// *** Remote Control Methods *********************************************
void PBotClass::parseIncomingBytes(void)
{
  int iByteCounter;

  PBotControl.parseIncomingBytes();

  if(PBotControl.blDPacketAvailable)
  {
    for(iByteCounter = 0; iByteCounter < 4; iByteCounter++)
    {
        strDProtocolID[iByteCounter] = PBotControl.strDProtocolID[iByteCounter];
    }

    for(iByteCounter = 0; iByteCounter < 5; iByteCounter++)
    {
        strCommandCode[iByteCounter] = PBotControl.strCommandCode[iByteCounter];
    }

    for(iByteCounter = 0; iByteCounter < 7; iByteCounter++)
    {
        strParam1[iByteCounter] = PBotControl.strParam1[iByteCounter];
    }

    for(iByteCounter = 0; iByteCounter < 7; iByteCounter++)
    {
        strParam2[iByteCounter] = PBotControl.strParam2[iByteCounter];
    }

    PBotControl.blDPacketAvailable = false;
    PBotControl.blDPacketReady = true;
 }

  blBufferOverflow = PBotControl.blBufferOverflow;
  blTerminationError = PBotControl.blTerminationError;
}

void PBotClass::interpretDProtocolCommands(void)
{
    PBotControl.interpretDProtocolCommands();
}

void PBotClass::resetDProtocol(void)
{
  PBotControl.resetDProtocol();
}

// *** To do... ***********************************************************
// Issues:
// <none reported yet>
//
// Enhancements:
// 1. code for tacho measurements
//
// ************************************************************************

// ************************************************************************
// End of PBot.cpp
// ************************************************************************
