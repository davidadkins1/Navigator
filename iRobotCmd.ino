/*********************************************************************
 *
 *      Robot Research Platform
 *
 *********************************************************************
 * FileName:        iRobotCmd.c
 * Dependencies:    See INCLUDES section below
 * Processor:       Arduino Mega 2560
 * Compiler:        Arduino 1.5.3
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * David Adkins       02/27/06    Original work started on PIC18 and FreeRTOS on
 *                                a stepping motor differential drive base.
 * David Adkins       08/24/13    Conversion to Arduino and cooperative multi-tsking
 ********************************************************************/
 
/** I N C L U D E S **********************************************************/
#include "oi.h"
#define comRX_BLOCK_TIME 1000L

/* Scheduler header files. */
//#include "RoboDetector_tasks.h"
#include "motor.h"
#include "behaviors.h"

/** Local Variables ********************************************************/

/** Global Variables ********************************************************/

/** Public Prototypes ***************************************/

/** External  P R O T O T Y P E S ***************************************/
boolean xUARTIOGetChar(byte port_number, char *pcRxedChar, long timeout);

/** P R I V A T E  P R O T O T Y P E S ***************************************/


/** Global Variables ********************************************************/

unsigned char g_songs[2][17];
unsigned char g_script[48];
unsigned char digitalOutput;


SENSORS sensorPacket =
{ 0,        // Packet ID:  7 - Bumps and Wheel Drops 
  0,        // Packet ID:  8 - Wall
  0,        // Packet ID:  9 - Cliff Left
  0,        // Packet ID: 10 - Cliff Front Left
  0,        // Packet ID: 11 - Cliff Front Right
  0,        // Packet ID: 12 - Cliff Right
  0,        // Packet ID: 13 - Virtual Wall
  0,        // Packet ID: 14 - Low Side Driver and Wheel Overcurrents
  0,        // Packet ID: 15 - Unused byte
  0,        // Packet ID: 16 - Unused byte
  255,      // Packet ID: 17 - Infrared byte 
  0,        // Packet ID: 18 - Buttons
  {0x0000},   // Packet ID: 19 - Distance Traveled
  {0x0000},   // Packet ID: 20 - Angle Travled
  0,        // Packet ID: 21 - Charging State
  0x4038,   // Packet ID: 22 - Battery Voltage (0x3840 = 14.4v)
  0x0CFE,   // Packet ID: 23 - Battery Current (0xFE0C = -500 mA)
  26,       // Packet ID: 24 - Battery Temperature
  0x6810,   // Packet ID: 25 - Battery Charge (0x1068 = 4200 mAh)
  0x6810,   // Packet ID: 26 - Battery Capacity
  0x0000,   // Packet ID: 27 - Wall Signal
  0x0000,   // Packet ID: 28 - Cliff Left Signal
  0x0000,   // Packet ID: 29 - Cliff Front Left Signal
  0x0000,   // Packet ID: 30 - Cliff Front Right Signal
  0x0000,   // Packet ID: 31 - Cliff Right Signal
  0x1F,     // Packet ID: 32 - Cargo Bay Digital Inputs
  0x0002,   // Packet ID: 33 - Cargo Bay Analog Signal (0x200 = null gyro value)
  0,        // Packet ID: 34 - Charging Sources Available
  OI_SAFE,  // Packet ID: 35 - OI Mode
  0,        // Packet ID: 36 - Song Number
  0,        // Packet ID: 37 - Song Playing
  0,        // Packet ID: 38 - Number of Stream Packets
  0x0000,   // Packet ID: 39 - Requested Velocity
  {0x0000},   // Packet ID: 40 - Requested Radius
  {0x0000},   // Packet ID: 41 - Requested Right Velocity
  {0x0000}    // Packet ID: 42 - Requested Left Velocity
};

/** Local Variables ********************************************************/
// Handle to the com port used by both tasks.
//static xComPortHandle xPort = NULL;

/** Public Prototypes ***************************************/

/** P R I V A T E  P R O T O T Y P E S ***************************************/

/********************************************************************
*    Function Name:  iRobotCmd                                      *
*    Return Value:   none                                           *
*    Parameters:     data: data to transmit                         *
*    Description:    iRobot command processor done via Bluetooth    *
********************************************************************/
void iRobotCmd( unsigned char cByteRxed )
{
	//
	// The command sent from the Host computer
	//
	unsigned char cmdByte, i;
	WORD_VAL leRight;
	WORD_VAL leLeft;
	#define turn_radius leRight
	DWORD_VAL leDistance;
	WORD_VAL input_heading;
  //enum CMD_STATE { PREFIX_1, PREFIX_2, CMD_READ, READ_SPEED_1, READ_SPEED_2 };

	cmdByte = cByteRxed;					// Store the command
	
	leRight.Val = 0;
	leLeft.Val = 0;
	leDistance.Val = 0;
	
	//if( (sensorPacket.sensor.ucOIMode == OIOff) && (cmdByte != CmdStart) )
	//{
	//    return;
	//}
			
	switch( cmdByte )
	{
		case CmdStart:
			sensorPacket.sensor.ucOIMode = OI_PASSIVE;
      #ifdef SERIAL_DEBUG
      //DebugPort.println("");
      #endif
		break;
		
		case CmdBaud:
      #ifdef SERIAL_DEBUG
      //DebugPort.println("");
      #endif
		break;
		
		case CmdControl:
			sensorPacket.sensor.ucOIMode = OI_SAFE;
    #ifdef SERIAL_DEBUG
    //DebugPort.println("");
    #endif
		break;
		
		case CmdSafe:
			sensorPacket.sensor.ucOIMode = OI_SAFE;
		break;
		
		case CmdFull:
			sensorPacket.sensor.ucOIMode = OI_FULL;
		break;
		
		case CmdSpot:
		break;
		
		case CmdClean:
		break;
		
		case CmdDemo:
		break;
		
		case CmdDrive:
    // Read velocity word
			xUARTIOGetChar( NULL, &leLeft.byte.HB , comRX_BLOCK_TIME );
    #ifdef SERIAL_DEBUG
    //DebugPort.write(leLeft.byte.HB);
    #endif
			sensorPacket.sensor.iRequestedLeftVelocity.byte.HB = leLeft.byte.HB;
			sensorPacket.sensor.iRequestedRightVelocity.byte.HB = leLeft.byte.HB;
			
			xUARTIOGetChar( NULL, &leLeft.byte.LB, comRX_BLOCK_TIME );
    #ifdef SERIAL_DEBUG
    //DebugPort.write(leLeft.byte.LB);
    //DebugPort.print(leLeft.sVal);
    //DebugPort.print(',');
    #endif
			sensorPacket.sensor.iRequestedLeftVelocity.byte.LB = leLeft.byte.LB;
			sensorPacket.sensor.iRequestedRightVelocity.byte.LB = leLeft.byte.LB;
			
    // Read radius word
			xUARTIOGetChar( NULL, &leRight.byte.HB, comRX_BLOCK_TIME );
    //#ifdef SERIAL_DEBUG
    //DebugPort.write(leRight.byte.HB);
    //Serial.write(leRight.byte.HB);
    //#endif
			sensorPacket.sensor.iRequestedRadius.byte.HB = leRight.byte.HB;
			
			xUARTIOGetChar( NULL, &leRight.byte.LB, comRX_BLOCK_TIME );
    //#ifdef SERIAL_DEBUG
    //Serial.write(leRight.byte.LB);
    //Serial.println(leRight.sVal);
    //#endif
			sensorPacket.sensor.iRequestedRadius.byte.LB = leRight.byte.LB;
	    //Serial.println(leLeft.sVal);

			vMotorDriveRadius( leRight.sVal, leLeft.sVal, 0, REMOTE_CONTROL );
		break;
		
		case CmdMotors:
		break;
		
		case CmdLeds:
		break;
		
		case CmdSong:
		    xUARTIOGetChar( NULL, &sensorPacket.sensor.ucSongNumber, comRX_BLOCK_TIME );
		    sensorPacket.sensor.ucSongNumber = cmdByte;
		    xUARTIOGetChar( NULL, &cmdByte, comRX_BLOCK_TIME );
		    xUARTIOGetChar( NULL, &g_songs[cmdByte][0], comRX_BLOCK_TIME );
			
			for( i = 1; i < g_songs[cmdByte][0] * 2 + 1; i++ )
			{
			  xUARTIOGetChar( NULL, &g_songs[cmdByte][i], comRX_BLOCK_TIME ); 
			}
		break;
		
		case CmdPlay:
			xUARTIOGetChar( NULL, &sensorPacket.sensor.ucSongPlaying, comRX_BLOCK_TIME );
			//xQueueSendToBack( xSongQueue, ( void * ) &sensorPacket.sensor.ucSongNumber, 0 );
			xUARTIOGetChar( NULL, &i, comRX_BLOCK_TIME );
			//xQueueSend( xSongQueue, ( void * ) &i, portMAX_DELAY  );
		break;
		
		case CmdSensors:
			i = xUARTIOGetChar( NULL, &cmdByte, comRX_BLOCK_TIME );
    #ifdef SERIAL_DEBUG
		//DebugPort.write(cmdByte);
    //DebugPort.print(cmdByte);
    #endif
			
			if( cmdByte == (unsigned char)0 )
			{
				vUARTIOWrite(sensorPacket.byte, Sen0Size );
				sensorPacket.sensor.iDistanceTraveled.Val = 0;          // Reset motion accumulators
				sensorPacket.sensor.iAngleTraveled.sVal = 0;             // after read
			}
			else if( cmdByte == (unsigned char)1 )
			{
				vUARTIOWrite(sensorPacket.byte, Sen1Size);
			}
			else if( cmdByte == (unsigned char)2 )
			{
        //read_current_rotatation();
				vUARTIOWrite(&sensorPacket.byte[SenIRChar], Sen2Size);
				sensorPacket.sensor.iDistanceTraveled.Val = 0;          // Reset motion accumulators
				sensorPacket.sensor.iAngleTraveled.sVal = 0;             // after read
			}
			else if( cmdByte == (unsigned char)3 )
			{
				vUARTIOWrite(&sensorPacket.byte[SenChargeState], Sen3Size);
			}
			else if( cmdByte == (unsigned char)4 )
			{
				vUARTIOWrite( &sensorPacket.byte[SenWallSig1], Sen4Size);
			}
			else if( cmdByte == (unsigned char)5 )
			{
				vUARTIOWrite(&sensorPacket.byte[SenOIMode], Sen5Size);
			}
			else if( cmdByte == (unsigned char)6 )
			{
				vUARTIOWrite(sensorPacket.byte, Sen6Size);
				sensorPacket.sensor.iDistanceTraveled.Val = 0;          // Reset motion accumulators
				sensorPacket.sensor.iAngleTraveled.sVal = 0;             // after read
			}
		break;
		
		case CmdDock:
		break;
		
		case CmdPWMMotors:
		break;
		
		case CmdDriveWheels:
			xUARTIOGetChar( NULL, &leRight.byte.HB, comRX_BLOCK_TIME );
      //#ifdef SERIAL_DEBUG
		  //DebugPort.write(leRight.byte.HB);
      //#endif
			sensorPacket.sensor.iRequestedRightVelocity.byte.HB = leRight.byte.HB;
			
	                xUARTIOGetChar( NULL, & leRight.byte.LB, comRX_BLOCK_TIME );
      //#ifdef SERIAL_DEBUG
		  //DebugPort.write(leRight.byte.LB);
      //#endif
			sensorPacket.sensor.iRequestedRightVelocity.byte.LB = leRight.byte.LB;
			
			xUARTIOGetChar( NULL, &leLeft.byte.HB, comRX_BLOCK_TIME );
      //#ifdef SERIAL_DEBUG
		  //DebugPort.write(leLeft.byte.HB);
      //#endif
			sensorPacket.sensor.iRequestedLeftVelocity.byte.HB = leLeft.byte.HB;

			xUARTIOGetChar( NULL, &leLeft.byte.LB, comRX_BLOCK_TIME );
      //#ifdef SERIAL_DEBUG
		  //DebugPort.write(leLeft.byte.LB);
      //#endif
			sensorPacket.sensor.iRequestedLeftVelocity.byte.LB = leLeft.byte.LB;

		  //if(0 != sensorPacket.sensor.iRequestedLeftVelocity.Val)
		  //{c
		  //}
		  //else
		  //{
		  //}
			
        #ifdef SERIAL_DEBUG
      //DebugPort.print(' ');
      //DebugPort.print(leRight.sVal);
      //DebugPort.print(' ');
      //DebugPort.println(leLeft.sVal);
        #endif
 
      vMotorDriveRequest( leLeft.sVal, leRight.sVal, 0, REMOTE_CONTROL );
		break;
		
		case CmdOutputs:
                        // TurtleBot bit 0 out is tied to bit 0 in for board detection
      xUARTIOGetChar( NULL, &digitalOutput, comRX_BLOCK_TIME );
                        
		break;
		
		case CmdSensorList:
		break;
		
		case CmdIRChar:
		break;
		
		case CmdScript:
			xUARTIOGetChar( NULL, &g_script[0], comRX_BLOCK_TIME );
			
			for( i = 1; i < (g_script[0] + 1); i++ )
			{
				xUARTIOGetChar( NULL, &g_script[i], comRX_BLOCK_TIME ); 
			}
		break;
		
		case PlayScript:
		    //xQueueSendToBack( xScriptQueue, ( void * ) script, 0 );
		break;
		
		case CmdDriveDistance:
			xUARTIOGetChar( NULL, &leLeft.byte.HB , comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedLeftVelocity.byte.HB = leLeft.byte.HB;
			sensorPacket.sensor.iRequestedRightVelocity.byte.HB = leLeft.byte.HB;
			
			xUARTIOGetChar( NULL, &leLeft.byte.LB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedLeftVelocity.byte.LB = leLeft.byte.LB;
			sensorPacket.sensor.iRequestedRightVelocity.byte.LB = leLeft.byte.LB;
			
			xUARTIOGetChar( NULL, &turn_radius.byte.HB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedRadius.byte.HB = turn_radius.byte.HB;
			
			xUARTIOGetChar( NULL, &turn_radius.byte.LB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedRadius.byte.LB = turn_radius.byte.LB;
			
			xUARTIOGetChar( NULL, &leDistance.byte.MB , comRX_BLOCK_TIME );
			xUARTIOGetChar( NULL, &leDistance.byte.UB , comRX_BLOCK_TIME );
			xUARTIOGetChar( NULL, &leDistance.byte.HB , comRX_BLOCK_TIME );
			xUARTIOGetChar( NULL, &leDistance.byte.LB , comRX_BLOCK_TIME );

			vMotorDriveRadius( turn_radius.Val, leLeft.Val, leDistance.Val, REMOTE_CONTROL );
		break;
		
		case TurnToHeading:
			xUARTIOGetChar( NULL, &leLeft.byte.HB , comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedLeftVelocity.byte.HB = leLeft.byte.HB;
			sensorPacket.sensor.iRequestedRightVelocity.byte.HB = leLeft.byte.HB;
			
			xUARTIOGetChar( NULL, &leLeft.byte.LB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedLeftVelocity.byte.LB = leLeft.byte.LB;
			sensorPacket.sensor.iRequestedRightVelocity.byte.LB = leLeft.byte.LB;
			
			xUARTIOGetChar( NULL, &turn_radius.byte.HB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedRadius.byte.HB = turn_radius.byte.HB;
			
			xUARTIOGetChar( NULL, &turn_radius.byte.LB, comRX_BLOCK_TIME );
			sensorPacket.sensor.iRequestedRadius.byte.LB = turn_radius.byte.LB;
			
			xUARTIOGetChar( NULL, &input_heading.byte.HB , comRX_BLOCK_TIME );
			xUARTIOGetChar( NULL, &input_heading.byte.LB , comRX_BLOCK_TIME );

			//vMotorTurnHeading( turn_radius.Val, leLeft.Val, input_heading.Val, REMOTE_CONTROL );
		break;
		
		default:
		break;
	}
}

