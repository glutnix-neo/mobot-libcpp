// ************************************************************************
// Filename: 	Telemetry.cpp
// Description:	Contains methods for repoting PBot status via D-Protocol
// Revision: 	v00.01.00
// Author:	    Efren S. Cruzat II
//	            (PhilRobotics.com, glutnix_neo of electronicslab.ph/forum)
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
// v00.01.00      20101216     Efren C.         Intiial Library Release
// ************************************************************************
#include <Telemetry.h>

// ************************************************************************
// *** Start of Telemetry Class *******************************************
// ************************************************************************

// *** Constructor ********************************************************
PBotTelemetryClass::PBotTelemetryClass(void)
{
  // null
}

void PBotTelemetryClass::begin(int iDBaudRate, char strDProtID[4], PBotClass *PBot)
{
  int iCounter;

  for(iCounter = 0; iCounter < 4; iCounter++)
  {
    strDProtocolID[iCounter] = strDProtID[iCounter];
  }
  begin(iDBaudRate, PBot);
}

void PBotTelemetryClass::begin(int iDBaudRate, PBotClass *PBot)
{
  setMyPBotObject(PBot);
  Serial.begin(iDBaudRate);
  ulPollIntTimer = millis();
}

void PBotTelemetryClass::setMyPBotObject(PBotClass *PBot)
{
  myPBot = PBot;
}

void PBotTelemetryClass::pollSensors(unsigned long ulPollingInterval)
{
  if((millis() - ulPollIntTimer) >= ulPollingInterval)
  {
    ulPollIntTimer = millis();

    pollWallSensors();
    pollLineSensors();
 //   pollTachometers(); esc.inactive
  }
}

// Public Methods
void PBotTelemetryClass::pollWallSensors(void)
{
  int iObstacleLocation;

  iObstacleLocation = myPBot->detectObstacle();
  reportWallSensors(iObstacleLocation);
}

void PBotTelemetryClass::pollLineSensors(void)
{
  int iLineLocation;

  iLineLocation = myPBot->detectLine();
  reportLineSensors(iLineLocation);
}

void PBotTelemetryClass::reportWallSensors(int iObstacleLocation)
{
  switch(iObstacleLocation)
  {
    case KI_OBJECT_NEARFRONT:
      printD(strDProtocolID, "rWSR", "NEAR", "FRONT");
      break;

    case KI_OBJECT_LEFTCENTER:
      printD(strDProtocolID, "rWSR", "LEFT", "CENTER");
      break;

    case KI_OBJECT_RIGHTCENTER:
      printD(strDProtocolID, "rWSR", "RIGHT", "CENTER");
      break;

    case KI_OBJECT_BOTHSIDES:
      printD(strDProtocolID, "rWSR", "BOTH", "SIDES");
      break;

    case KI_OBJECT_FARFRONT:
      printD(strDProtocolID, "rWSR", "FAR", "FRONT");
      break;

    case KI_OBJECT_LEFT:
      printD(strDProtocolID, "rWSR", "LEFT", "NULL");
      break;

    case KI_OBJECT_RIGHT:
      printD(strDProtocolID, "rWSR", "RIGHT", "NULL");
      break;

    default:
      printD(strDProtocolID, "rWSR", "NONE", "NULL");
      break;
  }
}

void PBotTelemetryClass::reportLineSensors(int iLineLocation)
{
  switch(iLineLocation)
  {
    case KI_OVER_LINE:
      printD(strDProtocolID, "rLSR", "OVER", "LINE");
      break;

    case KI_LINE_LEFTCENTER:
      printD(strDProtocolID, "rLSR", "LEFT", "CENTER");
      break;

    case KI_LINE_RIGHTCENTER:
      printD(strDProtocolID, "rLSR", "RIGHT", "CENTER");
      break;

    case KI_LINE_LEFT:
      printD(strDProtocolID, "rLSR", "LEFT", "NULL");
      break;

    case KI_LINE_RIGHT:
      printD(strDProtocolID, "rLSR", "RIGHT", "NULL");
      break;
    case KI_LINE_CENTER:
      printD(strDProtocolID, "rLSR", "CENTER", "NULL");
      break;

    default:
      printD(strDProtocolID, "rLSR", "NONE", "NULL");
      break;
  }
}

/* esc.inactive
void PBotTelemetryClass::pollTachometers()
{

}
*/

void PBotTelemetryClass::printD(char strDHeader[4], char strDCommandCode[5], char strDParam1[7], char strDParam2[7])
{
  Serial.print(strDHeader);                     // print D-Protocol Header
  Serial.print(" ");

  Serial.print(strDCommandCode);                // prints Command Code
  Serial.print(" ");

  if(strDParam1 != "NULL")
  {
    Serial.print(strDParam1);                   // prints Parameter1
    Serial.print(" ");

    if(strDParam2 != "NULL")
    {
      Serial.print(strDParam2);                 // prints Parameter2
      Serial.print(" ");
    }
  }
  Serial.println("");                           // prints D-Protocol Packet Terminator
}

// ************************************************************************
// End of Telemetry.cpp
// ************************************************************************

