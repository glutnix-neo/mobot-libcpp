// ************************************************************************
// Filename: 	DProtocol.cpp
// Description:	Listens to incoming UART transactions and parse D-Protocol commands
// Revision: 	v00.01.02
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
// v00.01.00      20110204     Efren C.         Initial Library Release
// v00.01.01       20110212     Efren C.         - fixed response to rLFT
//                                             				 (changed from right to left)
// v00.01.02      20120118      Efren C.         Bug Fixes:
//								- Changed 'F' to "F"
// ************************************************************************
#include <DProtocol.h>

// *** Constructor ********************************************************
DProtocolClass::DProtocolClass(void)
{
  //
}

void DProtocolClass::parseIncomingBytes(void)
{
  byte bRecievedByte;

  if (Serial.available() > 0)
  {
    ulDProtocolTimeout = millis();

    bRecievedByte = Serial.read();

    // Parse D-Protocol Header
    if(iByteCounter <= 3)
    {
      strRxDProtocolID[iByteCounter] = bRecievedByte;
      // Check if end of string
      if(strRxDProtocolID[iByteCounter] == ' ')
      {
        strRxDProtocolID[iByteCounter] =  '\0';
        iByteCounter = 3;
      }
    }
    // Parse Command Code
    else if(iByteCounter <= 8)
    {
      strRxCommandCode[iByteCounter - 4] = bRecievedByte;
      // Check if end of string
      if(strRxCommandCode[iByteCounter - 4] == ' ')
      {
        strRxCommandCode[iByteCounter - 4] =  '\0';
        iByteCounter = 8;
      }
    }
    // Parse Parameter 1
    else if(iByteCounter <= 15)
    {
      // Check if packet terminated (no Parameter1)
      if(bRecievedByte == 0x0D)
      {
        strRxDTerminator[0] = bRecievedByte;
        strRxParam1[0] = '\0';
        strRxParam2[0] = '\0';

        iByteCounter = 23;
      }

      strRxParam1[iByteCounter - 9] = bRecievedByte;

      if(strRxParam1[iByteCounter - 9] == ' ')
      {
        strRxParam1[iByteCounter - 9] =  '\0';
        iByteCounter = 15;
      }
    }
    // Parse Parameter 2
    else if(iByteCounter <= 22)
    {
      // Check if packet terminated (no Parameter2)
      if(bRecievedByte == 0x0D)
      {
        strRxDTerminator[0] = bRecievedByte;
        strRxParam2[0] = '\0';

        iByteCounter = 23;
      }

      strRxParam2[iByteCounter - 16] = bRecievedByte;

      // Check if end of string
      if(strRxParam2[iByteCounter - 16] == ' ')
      {
        strRxParam2[iByteCounter - 16] =  '\0';
        iByteCounter = 22;
      }
    }
    // Wait for Packet Terminator
    else if(iByteCounter <= 24)
    {
      strRxDTerminator[iByteCounter - 23] = bRecievedByte;

      if(iByteCounter == 24)
      {
        if(strRxDTerminator[0] == 0x0D && strRxDTerminator[1] == 0x0A)
        {
          // Transfer Bytes to Working Registers
          for(iByteCounter = 0; iByteCounter < 4; iByteCounter++)
          {
            strDProtocolID[iByteCounter] = strRxDProtocolID[iByteCounter];
          }

          for(iByteCounter = 0; iByteCounter < 5; iByteCounter++)
          {
            strCommandCode[iByteCounter] = strRxCommandCode[iByteCounter];
          }

          for(iByteCounter = 0; iByteCounter < 7; iByteCounter++)
          {
            strParam1[iByteCounter] = strRxParam1[iByteCounter];
          }

          for(iByteCounter = 0; iByteCounter < 7; iByteCounter++)
          {
            strParam2[iByteCounter] = strRxParam2[iByteCounter];
          }

          blDPacketAvailable = true;

          // clear Rx buffers
          resetDProtocol();
        }
        // Error on Termination, clear Rx Buffers
        else
        {
          blTerminationError = true;
          Serial.println("#Terminator Error");
        }
      }
    }
    // Buffer Overflow
    else
    {
      blBufferOverflow = true;
      Serial.println("#Buffer Overflow");
      blDPacketAvailable = false;
    }

    iByteCounter++;
  }

  // Check for Timeout
  if((millis() - ulDProtocolTimeout) >= KUL_DPROTOCOL_TIMEOUT)
  {
    ulDProtocolTimeout = millis();
    resetDProtocol();
  }
}

void DProtocolClass::resetDProtocol(void)
{
    ulDProtocolTimeout = millis();
    iByteCounter = 0;

    strRxDProtocolID[0] = '\0';
    strRxCommandCode[0] = '\0';
    strRxParam1[0] = '\0';
    strRxParam2[0] = '\0';
    strRxDTerminator[0] = '\0';
	#if defined(ARDUINO) && ARDUINO >= 100
	#else
    Serial.flush();
	#endif
}

// Interpret DProtocol
void DProtocolClass::interpretDProtocolCommands(void)
{
    if(blDPacketReady)
    {
        if(checkPBotMotionCommands())
        {
            // null
        }
        else if(checkSensorCommands())
        {
            // null
        }
        /*
            Insert other D-Protocol checking here
        */
        else // default: stopMotors
        {
          Serial.println("#Command Not Supported");
          printD(strDProtocolID, "cNSP", "NOT", "KNOWN");
          bl_reply_supressed = true;

          myPBot->stopMotors();
        }

        // echo recieved bytes
        if(bl_reply_supressed)
        {
          bl_reply_supressed = false;
        }
        else
        {
          Serial.print(myPBot->strDProtocolID);
          Serial.print(' ');
          Serial.print(myPBot->strCommandCode);
          Serial.print(' ');
          Serial.print(myPBot->strParam1);
          Serial.print(' ');
          Serial.print(myPBot->strParam2);
          Serial.print(' ');
          Serial.println("");
        }

        blDPacketReady = false;
    }
}

boolean DProtocolClass::checkSensorCommands(void)
{
    // read command reply
    if(String(myPBot->strCommandCode) == "rWSC")
    {
      Serial.println("#Wall Sensors Status");
      Serial.print("D00");
      reportWallSensors(myPBot->iObstacleLocation);
      bl_reply_supressed = true;

      return true;
    }
    else if(String(myPBot->strCommandCode) == "rLSC")
    {
      Serial.println("#Line Sensors Status");
      Serial.print("D00");
      reportLineSensors(myPBot->iLineLocation);
      bl_reply_supressed = true;

      return true;
    }

    return false;
}

boolean DProtocolClass::checkPBotMotionCommands(void)
{
    boolean bl_Temp;

    // low level mobot control
    if(String(myPBot->strCommandCode) == "rRMT")
    {
      Serial.println("#Run Right Motor");
      if(String(myPBot->strParam2) == "F")
      {
        bl_Temp = 0;
      }
      else
      {
        bl_Temp = 1;
      }

      myPBot->runRightMotor(stringToByte(myPBot->strParam1), bl_Temp);

      return true;
    }
    else if(String(myPBot->strCommandCode) == "rLMT")
    {
      Serial.println("#Run Left Motor");
      if(String(myPBot->strParam2) == "F")
      {
        bl_Temp = 0;
      }
      else
      {
        bl_Temp = 1;
      }

      myPBot->runLeftMotor(stringToByte(myPBot->strParam1), bl_Temp);

      return true;
    }
    else if(String(myPBot->strCommandCode) == "sRMT")
    {
      Serial.println("#Stop Right Motor");
      myPBot->stopRightMotor();

      return true;
    }
    else if(String(myPBot->strCommandCode) == "sLMT")
    {
      Serial.println("#Stop Left Motor");
      myPBot->stopLeftMotor();

      return true;
    }

    // high level mobot control
    else if(String(myPBot->strCommandCode) == "mFWD")
    {
      Serial.println("#Move Forward");
      myPBot->moveForward(stringToByte(myPBot->strParam1));

      return true;
    }
    else if(String(myPBot->strCommandCode) == "mBCK")
    {
      Serial.println("#Move Backward");
      myPBot->moveBackward(stringToByte(myPBot->strParam1));

      return true;
    }
    else if(String(myPBot->strCommandCode) == "tRGT")
    {
      Serial.println("#Turn Right");
      myPBot->turnRight(stringToByte(myPBot->strParam1));

      return true;
    }
    else if(String(myPBot->strCommandCode) == "tLFT")
    {
      Serial.println("#Turn Left");
      myPBot->turnLeft(stringToByte(myPBot->strParam1));

      return true;
    }
    else if(String(myPBot->strCommandCode) == "rRGT")
    {
      Serial.println("#Rotate Right");
      myPBot->rotateRight(stringToByte(myPBot->strParam1));

      return true;
    }
    else if(String(myPBot->strCommandCode) == "rLFT")
    {
      Serial.println("#Rotate Left");
      myPBot->rotateLeft(stringToByte(myPBot->strParam1));

      return true;
    }
    else if(String(myPBot->strCommandCode) == "sPBT")
    {
      Serial.println("#Stop PBot");
      myPBot->stopMotors();

      return true;
    }

    return false;
}

byte DProtocolClass::stringToByte(char cNumeral[])
{
  byte bTemp = 0;
  int  iTemp1,iTemp2;

  for(iTemp1 = 0; iTemp1 < 10; iTemp1++)
  {
      if(cNumeral[iTemp1] == '\0')
      {
          break;
      }
  }
  iTemp2 = iTemp1;

  //Serial.println(iTemp1, DEC); esc.test debug code

  // hundreds digit
  if(iTemp1 == 3)
  {
      bTemp = numberLookup(cNumeral[iTemp2 - iTemp1]) * 100;
      iTemp1--;
  }
  // tens digit
  if(iTemp1 == 2)
  {
      bTemp += numberLookup(cNumeral[iTemp2 - iTemp1]) * 10;
      iTemp1--;
  }
  // ones digit
  if(iTemp1 == 1)
  {
      bTemp += numberLookup(cNumeral[iTemp2 - iTemp1]);
  }

  //Serial.println(bTemp, DEC); esc.test debug code

  return bTemp;
}

byte DProtocolClass::numberLookup(char cNumeral)
{
  if(cNumeral == '1')
  {
    return 1;
  }
  else if(cNumeral == '2')
  {
    return 2;
  }
  else if(cNumeral == '3')
  {
    return 3;
  }
  else if(cNumeral == '4')
  {
    return 4;
  }
  else if(cNumeral == '5')
  {
    return 5;
  }
  else if(cNumeral == '6')
  {
    return 6;
  }
  else if(cNumeral == '7')
  {
    return 7;
  }
  else if(cNumeral == '8')
  {
    return 8;
  }
  else if(cNumeral == '9')
  {
    return 9;
  }
  else
  {
    return 0;
  }
}


// ************************************************************************
// End of DProtocol.cpp
// ************************************************************************
