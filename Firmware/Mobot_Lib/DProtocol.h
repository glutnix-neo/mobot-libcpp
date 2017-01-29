// ************************************************************************
// Filename: 	DProtocol.h
// Description:	Listens to incoming UART transactions and parse D-Protocol commands
// Revision: 	v00.01.01
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
// v00.01.00      20110204     Efren C.         Intiial Library Release
// v00.01.01       20120118     Efren C.         Enhancements:
//								- removed included files
//									WProgram.h
//									WConstants.h
//									LowLevelMobot.h
//									PBot.h
// ************************************************************************
//#include <WProgram.h>
//#include <WConstants.h>
//#include <LowLevelMobot.h>
//#include <PBot.h>
#include <Telemetry.h>

#ifndef DProtocol_h
#define DProtocol_h

class DProtocolClass : public PBotTelemetryClass
{
  private:
  // *** private variables **********************************************
  // Recieve Buffer
  char strRxDProtocolID[4];
  char strRxCommandCode[5];
  char strRxParam1[7];
  char strRxParam2[7];
  char strRxDTerminator[3];

  boolean bl_reply_supressed;
  unsigned long ulDProtocolTimeout;
  int  iByteCounter;

  // *** private functions **********************************************
  byte stringToByte(char cNumeral[]);
  byte numberLookup(char cNumeral);

  public:
  // *** public variables ***********************************************
  // Transmit Buffer
  //char strTxDProtocolID[4];
  //char strTxCommandCode[5];
  //char strTxParam1[7];
  //char strTxParam2[7];

  // Working Buffer
  char strDProtocolID[4];
  char strCommandCode[5];
  char strParam1[7];
  char strParam2[7];

  boolean blDPacketReady;
  boolean blDPacketAvailable;
  boolean blBufferOverflow;
  boolean blTerminationError;

  // *** public functions ***********************************************
  DProtocolClass(void);

  void parseIncomingBytes(void);
  void resetDProtocol(void);

  void interpretDProtocolCommands(void);
  boolean checkSensorCommands(void);
  boolean checkPBotMotionCommands(void);
};

#endif
// ************************************************************************
// End of DProtocol.h
// ************************************************************************
