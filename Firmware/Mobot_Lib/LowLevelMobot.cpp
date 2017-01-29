// ************************************************************************
// Filename: 	LowLevelMobot.cpp
// Description:	Low Level Mobot methods and definitions. This contains
//              portable classes. This can easily be modified to match
//              hardware requirements
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
// v00.01.00      20101216     Efren C.         	Intiial Library Release
// v00.01.01       20120118     Efren C.        	Enhancements:
//								 - removed included files
//									WProgram.h
//									WConstants.h
// ************************************************************************
//#include <WProgram.h>
//#include <WConstants.h>
#include "LowLevelMobot.h"

// *** Constructor ********************************************************
LowLevelMobotClass::LowLevelMobotClass()
{
    // null
}

// *** Low Level Motor Control Functions **********************************
void LowLevelMobotClass::runRightMotor(byte bSpeed, boolean blDirection)
{
  digitalWrite(KI_RIGHTMOTOR_DIRCTRL, blDirection);
  analogWrite(KI_RIGHTMOTOR_RUNCTRL, bSpeed);
}

void LowLevelMobotClass::runLeftMotor(byte bSpeed, boolean blDirection)
{
  digitalWrite(KI_LEFTMOTOR_DIRCTRL, blDirection);
  analogWrite(KI_LEFTMOTOR_RUNCTRL, bSpeed);
}

void LowLevelMobotClass::stopRightMotor(void)
{
  digitalWrite(KI_RIGHTMOTOR_DIRCTRL, LOW);
  analogWrite(KI_RIGHTMOTOR_RUNCTRL, 0);
}

void LowLevelMobotClass::stopLeftMotor(void)
{
  digitalWrite(KI_LEFTMOTOR_DIRCTRL, LOW);
  analogWrite(KI_LEFTMOTOR_RUNCTRL, 0);
}

// *** Low Level Sensor Functions *****************************************
boolean LowLevelMobotClass::readSensorOutput(int iSensorPin, boolean blPolarity)
{
    if(blPolarity)
    {
        return !digitalRead(iSensorPin);        // active low
    }
    else
    {
        return digitalRead(iSensorPin);         // active high
    }
}

unsigned long LowLevelMobotClass::getRightTachoPeriod(boolean blActiveEdge)  // 1 - rising edge, 0 - falling edge
{
    static  boolean blPrevTachoState;
            boolean blTachoState;
            boolean blEdgeDetected;
    static  unsigned long ulPrevEdgeMarkTimer;
            unsigned long ulEdgeMarkTimer;                                  // will be initialized and used only when edge is detected
    static  unsigned long ulPollTimeoutTimer;
    static  unsigned long ulTachoPeriod;

    blEdgeDetected = false;

    // poll tacho pin
    blTachoState = readSensorOutput(KI_RIGHTWHEEL_TACHO, !blActiveEdge);

    // detect edge
    if(!blPrevTachoState && blTachoState)
    {
        blEdgeDetected = true;
        ulEdgeMarkTimer = millis();
        ulPollTimeoutTimer = ulEdgeMarkTimer;
    }

    // check timeoout
    if(!blEdgeDetected)
    {
        if((millis() - ulPollTimeoutTimer) >= KUL_TACHOPOLLTIMEOUT)
        {
            blEdgeDetected = false;
            ulTachoPeriod = 0;
        }
    }
    // compute tacho period
    else
    {
        blEdgeDetected = false;

        // IIR 1/4 new, 3/4 old
        ulTachoPeriod >>= 2;
        ulTachoPeriod *= 3;
        ulTachoPeriod += ((ulEdgeMarkTimer - ulPrevEdgeMarkTimer) >> 2);

        ulPrevEdgeMarkTimer = ulEdgeMarkTimer;
    }

    blPrevTachoState = blTachoState;

    return ulTachoPeriod;
}

unsigned long LowLevelMobotClass::getLeftTachoPeriod(boolean blActiveEdge)  // 1 - rising edge, 0 - falling edge
{
    static  boolean blPrevTachoState;
            boolean blTachoState;
            boolean blEdgeDetected;
    static  unsigned long ulPrevEdgeMarkTimer;
            unsigned long ulEdgeMarkTimer;                                  // will be initialized and used only when edge is detected
    static  unsigned long ulPollTimeoutTimer;
    static  unsigned long ulTachoPeriod;

    blEdgeDetected = false;

    // poll tacho pin
    blTachoState = readSensorOutput(KI_LEFTWHEEL_TACHO, !blActiveEdge);

    // detect edge
    if(!blPrevTachoState && blTachoState)
    {
        blEdgeDetected = true;
        ulEdgeMarkTimer = millis();
        ulPollTimeoutTimer = ulEdgeMarkTimer;
    }

    // check timeoout
    if(!blEdgeDetected)
    {
        if((millis() - ulPollTimeoutTimer) >= KUL_TACHOPOLLTIMEOUT)
        {
            blEdgeDetected = false;
            ulTachoPeriod = 0;
        }
    }
    // compute tacho period
    else
    {
        blEdgeDetected = false;

        // IIR 1/4 new, 3/4 old
        ulTachoPeriod >>= 2;
        ulTachoPeriod *= 3;
        ulTachoPeriod += ((ulEdgeMarkTimer - ulPrevEdgeMarkTimer) >> 2);

        ulPrevEdgeMarkTimer = ulEdgeMarkTimer;
    }

    blPrevTachoState = blTachoState;

    return ulTachoPeriod;
}

void LowLevelMobotClass::pollWallSensors(boolean *blObstacleSensorPolarity)
{
  blWallDetectedonLeft = readSensorOutput(KI_LEFT_WALLSENSOR, *blObstacleSensorPolarity);
  blWallDetectedonCenter = readSensorOutput(KI_CENTER_WALLSENSOR, *blObstacleSensorPolarity);
  blWallDetectedonRight = readSensorOutput(KI_RIGHT_WALLSENSOR, *blObstacleSensorPolarity);
}

void LowLevelMobotClass::pollLineSensors(boolean *blLineSensorPolarity)
{
  blLineDetectedonLeft = readSensorOutput(KI_LEFT_LINESENSOR, *blLineSensorPolarity);
  blLineDetectedonCenter = readSensorOutput(KI_CENTER_LINESENSOR, *blLineSensorPolarity);
  blLineDetectedonRight = readSensorOutput(KI_RIGHT_LINESENSOR, *blLineSensorPolarity);
}

// *********************************************************************
// End of LowLevelMobot.cpp
// *********************************************************************
