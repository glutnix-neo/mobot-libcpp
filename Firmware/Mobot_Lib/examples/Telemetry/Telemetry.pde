// ************************************************************************/
// Filename: 	Telemetry.pde
// Description:	Demonstrate P-Bot telemetry through UART using D-Protocol
//
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
// v00.00.01       20101115    Efren C.      Sketch Initial Release 
//
// v01.00.00       20101216    Efren C.      Enhancements:      
//                                           - separate begin method
// ************************************************************************
#include <PBot.h>

PBotClass Telemetry;

#define KUL_POLLING_INTERVAL  1000                       // sensor polling and peporting interval in mS
unsigned long ulInterval;
                                                         
void setup()
{
  Telemetry.begin(SUMO,WHITE,BLACK,true,9600);           // Sumo Robot Mode, 
                                                         // Detect White Obstacles, 
                                                         // Detect Black Lines
                                                         // Enable Telemetry
                                                         // 9600bps
}
 
void loop()
{
  Telemetry.pollSensors(KUL_POLLING_INTERVAL);           // poll and report status every 1S
}
                                                         
// ************************************************************************
// End of Telemetry.pde
// ************************************************************************                                                        
