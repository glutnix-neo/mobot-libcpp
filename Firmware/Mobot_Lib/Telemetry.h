// ************************************************************************
// Filename: 	Telemetry.h
// Description:	Contains methods for repoting PBot status via D-Protocol
// Revision: 	v00.02.01
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
// v00.02.00      20110204     Efren C.         reportWallSensors and reportLineSensors
//                                                  		made public
// v00.02.01       20120118     Efren C.        	Enhancements:
//								 - removed included files
//									WProgram.h
//									WConstants.h
//									LowLevelMobot.h
// ************************************************************************
//#include <WProgram.h>
//#include <WConstants.h>
//#include <LowLevelMobot.h>
#include <PBot.h>

#ifndef PBotTelemetry_h
#define PBotTelemetry_h

#define KUL_DPROTOCOL_TIMEOUT    100  // 1ms resolution

// forward reference
class PBotClass;

class PBotTelemetryClass
{
  private:
    // *** private variables **********************************************

    // *** private functions **********************************************

  public:
    // *** public variables ***********************************************
    PBotClass *myPBot;

    unsigned long  ulPollIntTimer;
    char strDProtocolID[4];

    // *** public functions ***********************************************
    PBotTelemetryClass(void);

    void setMyPBotObject(PBotClass *PBot);          // set a Pbot instance inside the telemetry class

    void begin(int iDBaudRate, PBotClass *PBot);
    void begin(int iDBaudRate, char strDProtID[4], PBotClass *PBot);

    void pollSensors(unsigned long ulPollingInterval);

    void pollWallSensors(void);
    void pollLineSensors(void);

    void reportWallSensors(int iObstacleLocation);
    void reportLineSensors(int iLineLocation);

    void printD(char strDHeader[4], char strDCommandCode[5], char strDParam1[7], char strDParam2[7]);
};

#endif
// *********************************************************************
// End of Telemetry.h
// *********************************************************************




