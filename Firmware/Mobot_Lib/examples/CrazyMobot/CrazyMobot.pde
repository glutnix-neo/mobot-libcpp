// ************************************************************************/
// Filename: 	CrazyMobot.pde
// Description:	Demonstrate Low Level and High Level Motor Control functions
//		for PBot Mobile Robot Kit (and compatible mobots)
//		visit "http://www.e-gizmo.com/KIT/P-BOT.htm" for more info.
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
// ************************************************************************
// FW Version      Date        Author        Description
// v00.01.00       20101126    Efren C.      Sketch Initial Release 
//
// v01.00.00       20101216    Efren C.      Enhancements:      
//                                           - separate begin method
// ************************************************************************
#include <PBot.h>

PBotClass CrazyMobot;

#define KUL_SWITCHSTATE_INTERVAL    1500                 // 1.5S to switch to next state
unsigned long ulDuration;

void setup()
{
  CrazyMobot.begin(SUMO,WHITE,BLACK);                    // Sumo Robot Mode, 
                                                         // Detect White Obstacles, 
                                                         // Detect Black Lines  
  // Initialize Timers
  ulDuration = millis();  
}

void loop()
{
  int iState;
  
  while(1)                                               // Bug workaround for Arduino v0021
  {
    // State Machine
    switch (iState)
    {
      case 0: // Right Motor Running
        CrazyMobot.runRightMotor(FULLSPEED, FORWARD);
        break;
        
      case 1: // Right Motor Reverse
         CrazyMobot.runRightMotor(FULLSPEED, REVERSE);
        break;  
        
       case 2: // Left Motor Running
        CrazyMobot.runLeftMotor(FULLSPEED, FORWARD);
        break;
        
      case 3: // Left Motor Reverse
        CrazyMobot.runLeftMotor(FULLSPEED, REVERSE);  
        break;      
        
      case 4: // Forward @ Mid Speed
        CrazyMobot.moveForward(MIDSPEED); 
        break;
        
      case 5: // Forward @ Low Speed
        CrazyMobot.moveForward(LOWSPEED); 
        break;  
        
      case 6: // Reverse @ Low Speed
        CrazyMobot.moveBackward(LOWSPEED); 
        break;
        
      case 7: // Rotate Left
        CrazyMobot.rotateLeft(MIDSPEED);
        break;
        
      case 8: // Rotate Right
        CrazyMobot.rotateRight(MIDSPEED);
        break;
        
      case 9: // Turn Left
        CrazyMobot.turnLeft(MIDSPEED);
        break;
        
      case 10: // Turn Right
        CrazyMobot.turnRight(MIDSPEED);
        break;
                    
      case 11: // stop
        CrazyMobot.rotateRight(FULLSPEED);
        delay(500);
        CrazyMobot.rotateLeft(FULLSPEED);
        delay(500);
        CrazyMobot.stopMotors();
        break;
        
      default: // stop
        CrazyMobot.stopMotors();
        break;      
    }
    
    // Scheduler
    if((millis() - ulDuration) >= KUL_SWITCHSTATE_INTERVAL)
    {
      ulDuration = millis();

      if(iState <= 10)
      {
        iState++;
      }
      else
      {
        iState = 0;
      }
    }
  }
}

// ************************************************************************
// End of CrazyMobot.pde
// ************************************************************************
