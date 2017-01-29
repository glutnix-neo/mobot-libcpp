// ************************************************************************/
// Filename: 	SumoRobot.pde
// Description:	Sumo Robot Demo Sketch for E-Gizmo's P-Bot
// Revision: 	v01.00.00
// Author:	Efren S. Cruzat II
//		(PhilRobotics.com, glutnix_neo of electronicslab.ph/forum)
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
// ************************************************************************/
// FW Version      Date        Author        Description
// v00.00.03       20101119    Efren C.      Initial Library Release
//
// v00.01.00       20101126    Efren C.      Bug Fixes:
//                                           - Adjusted Delay Timings
//                                           - Modified wall detection scheme
//                                           Enhancements:
//                                           - Randomly select which direction
//                                             to rotate when encountered a line
//                                           - Used the predefined P-Bot mode
//                                             on object creation
//
// v01.00.00       20101216    Efren C.      Enhancements:      
//                                           - separate begin method
//
// ************************************************************************
#include <PBot.h>

PBotClass  SumoMobot;

// Validation Delays
#define KUL_LINESENSOR_DEBOUNCE_MS    10

unsigned long ulLineSensorDebounce;
boolean blLineDetected;

void setup()
{
  SumoMobot.begin(SUMO,WHITE,BLACK);       // Sumo Robot Mode, 
                                           // Detect White Wall, 
                                           // Detect Black Lines
  // Initialize Timers
  ulLineSensorDebounce = millis();
}

void loop()
{
  int iLineState;
  int iWallState;
  boolean blRotateRight;
  boolean blRandomLocked;
  
  // Read Line Sensors
  iLineState = SumoMobot.detectLine();
  
  // Line Detection Debounce
  if(iLineState != NO_LINE)
  {
    if((millis() - ulLineSensorDebounce) >= KUL_LINESENSOR_DEBOUNCE_MS)            // validation delay
    {
      blLineDetected = true;
    }
  }
  else
  {
    ulLineSensorDebounce = millis();
    blLineDetected = false;
  }
    
  // Detect Obstacles
  iWallState = SumoMobot.detectObstacle();

  // Randomly Select Rotation
  if(random(256) >= 85 && !blRandomLocked)
  {
    blRotateRight = true;
  }
  else
  {
    blRotateRight = false;
  }
  
  // Move P-Bot
  if(blLineDetected)
  {
    SumoMobot.moveBackward(MIDSPEED);
    delay(200);
    
    if(blRotateRight)
    {
      SumoMobot.rotateRight(MIDSPEED);
    }
    else
    {
      SumoMobot.rotateLeft(MIDSPEED);
    }
    blRandomLocked = true;
    delay(300);
  }  
  // right
  else if(iWallState == OBJECT_RIGHTCENTER)
  {
    SumoMobot.turnRight(MIDSPEED);
    delay(300);
  }
  // left
  else if(iWallState == OBJECT_LEFTCENTER)
  {
    SumoMobot.turnLeft(MIDSPEED);
    delay(300);    
  }  
  // far right
  else if(iWallState == OBJECT_RIGHT)
  {
    SumoMobot.rotateRight(MIDSPEED);
  }
  // far left
  else if(iWallState == OBJECT_LEFT)
  {
    SumoMobot.rotateLeft(MIDSPEED);
  }
  // near front
  else if(iWallState == OBJECT_NEARFRONT)
  {
    SumoMobot.moveForward(FULLSPEED);
  }  
  else
  {
    SumoMobot.moveForward(LOWSPEED);
    blRandomLocked = false;
  }
}

// ************************************************************************
// End of SumoRobot.pde
// ************************************************************************
