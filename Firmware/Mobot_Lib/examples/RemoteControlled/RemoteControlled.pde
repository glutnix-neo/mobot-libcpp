// ************************************************************************/
// Filename: 	RemoteControlled.pde
// Description:	Demonstrate P-Bot telemetry through UART using D-Protocol
//
// Revision: 	v00.01.00
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
// FW Version      Date        Author         Description
// v00.01.00       20110204    Efren C.       Sketch Initial Release 
//
// ************************************************************************
#include <PBot.h>

PBotClass  RemoteControl;

// Bluetooth Address (Little Endian)
const byte BT_Addr[6] = {0x12, 0x13, 0x23, 0x17, 0x08, 0x00};                                               // any 6 bytes hex digits
const byte BT_Alias[15] = {"P-BOT"};                                                                        // max of 15 characters
const byte BT_SecurityCode[4] = {'1', '2', '3', '4' };                                                      // 4 digit code (ASCII)

void setup()
{
  initEGizmoBTModule();                                                                                     // send commands to establish contact
  
  RemoteControl.begin(REMOTECONTROL,WHITE,BLACK,0,9600);              
                                                // Remote Control Mode, 
                                                // Detect White Obstacles, 
                                                // Detect Black Lines 
                                                // Telemetry Disabled
                                                // 9600bps baudrate 
}
 
void loop()
{
    // Listen to D Commands
    RemoteControl.parseIncomingBytes();
    RemoteControl.detectObstacle();
    RemoteControl.detectLine();
    RemoteControl.interpretDProtocolCommands();
}

// Initializes E-Gizmo Bluetooth module to be ready for data transfer, Default data rate is 9600bps
void initEGizmoBTModule(void) 
{
  // Bluetooth Serial Initialization Commands
  byte BT_InitBuff_Cmd[13] = {0x02, 0x52, 0x27, 0x06, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03}; // 6-11
  byte BT_SetAlias_Cmd[24] = {0x02, 0x52, 0x04, 0x11, 0x00, 0x67, 0x10, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x03};
  byte BT_SetSecurity_Cmd[12] = {0x02, 0x52, 0x17, 0x05, 0x00, 0x6E, 0x04, 0x31, 0x32, 0x33, 0x34, 0x03};
  byte BT_StartCore_Cmd[7] = {0x02, 0x52, 0x66, 0x00, 0x00, 0xB8, 0x03};  
  
  byte i8TempCntr0, i8TempCntr1;
  
  Serial.begin(9600);                                            // Set UART Baudrate to 9600bps  
  
  // Parse Blutetooth Address
  i8TempCntr1 = 0;  
  for(i8TempCntr0 = 6; i8TempCntr0 < 12; i8TempCntr0++)
  {
    BT_InitBuff_Cmd[i8TempCntr0] = BT_Addr[i8TempCntr1];
    i8TempCntr1++;    
  }
  
  // Set Blutetooth Address
  Serial.write(BT_InitBuff_Cmd,13);
  delay(250);

  // Parse Bluetooth Alias
  i8TempCntr1 = 0;  
  for(i8TempCntr0 = 7; i8TempCntr0 < 22; i8TempCntr0++)
  {
    BT_SetAlias_Cmd[i8TempCntr0] = BT_Alias[i8TempCntr1];
    i8TempCntr1++;    
  } 

  // Set Bluetooth Alias
  Serial.write(BT_SetAlias_Cmd,24); 
  delay(250);
  
  // Parse Bluetooth Security Code
  i8TempCntr1 = 0;  
  for(i8TempCntr0 = 7; i8TempCntr0 < 11; i8TempCntr0++)
  {
    BT_SetSecurity_Cmd[i8TempCntr0] = BT_SecurityCode[i8TempCntr1];
    i8TempCntr1++;    
  } 
  
  // Set Bluetooth Security Code
  Serial.write(BT_SetSecurity_Cmd,12);
  delay(250);
 
  // Start Bluetooth Core
  Serial.write(BT_StartCore_Cmd,7);
  delay(500);
  
  Serial.println(' ');
  Serial.flush();
}
// ************************************************************************
// End of RemoteControlled.pde
// ************************************************************************                                                        
