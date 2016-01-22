/*******************************************************************************
 * FileName:    PDACMD.ino
 * Product:	ClipBoardBot
 * Processor:   Arduino AVR
 * Purpose:	Process simple commands
 *
 * (c)2013 David Adkins
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Revision History: 
 *
 * Revision 0.1 09/13 by David Adkins: 
 * 		1. Creation of code base
 ******************************************************************************/
#include <SoftwareSerial.h>   //Software Serial Port

enum CMD_STATE { PREFIX_2, CMD_READ, READ_SPEED_1, READ_SPEED_2 };
//static char BANNER[ ] = { "Ready!\r\n" };
static enum CMD_STATE cmdState = PREFIX_2; 
extern SoftwareSerial blueToothSerial;
/********************************************************************
*    Function Name:  vPDACmd                                        *
*    Return Value:   true if command processing complete            *
*    Parameters:     data: data to transmit                         *
*    Description:    Host command processor done via Bluetooth	    *
********************************************************************/
boolean vPDACmd( unsigned char cByteRxed)
{
  //
  // The command sent from the Host computer
  //
  unsigned char cmdByte;
  //int speedL, speedR;
  switch(cmdState)
  {
    case PREFIX_2:
      if( cByteRxed == 0x07 )
      {
        //#ifdef SERIAL_DEBUG
        //DebugPort.println("PDA command 2");
        //#endif
        cmdState = CMD_READ;
      }
      else
      {
        return true;
      }
    break;

    case CMD_READ:
      cmdByte = cByteRxed;					// Store the command
      //SerialPort.println("Hello");
      //SerialPort.println(cmdByte);
      //Forward
      if( 'F' == cmdByte )					// Both motors forward
      {
        vMotorDriveRequest( 100, 100, 0, REMOTE_CONTROL );
        cmdState = PREFIX_2;
        return true;
      }
      //Reverse
      else if( 'R' == cmdByte )				// Both motors reverse
      {
        vMotorDriveRequest( -100, -100, 0, REMOTE_CONTROL );
        cmdState = PREFIX_2;
        return true;
      }
      // Motor status
      else if( 'M' == cmdByte )				// Both motors reverse
      {
        //string_size = sprintf(motor_string,(const far rom char *)"%03u,%05u,%05u,%+05i,%+05i\n", current_heading(), uiLeftEMF(), uiRightEMF(), iVelocityLeft(), iVelocityRight() );
        //motor_string[string_size] = 0;
        //vUARTIOputsBT(motor_string);
        cmdState = PREFIX_2;
        return true;
      }
      // Autonomous
      else if( 'a' == cmdByte )
      {
        vReleaseControl( REMOTE_CONTROL );
        cmdState = PREFIX_2;
        return true;
      }
      else if( 'g' == cmdByte )
      {
        //
        // Read the analog value from the current sensor
        //
        //
        // Send the values to the PDA
        //
		
        //vUARTIOWriteBT( uiLeftCurrent( ) >> 2 );	
        //vUARTIOWriteBT( (char)(uiRightCurrent( ) >> 2) );	
	//vUARTIOWriteBT( (iVelocityLeft( ) & 0xff) );	
	//vUARTIOWriteBT( (char)(iVelocityLeft( ) >> 8 & 0xff) );	
	//vUARTIOWriteBT( (char)(iVelocityRight( ) & 0xff) );	
	//vUARTIOWriteBT( (char)(iVelocityRight( ) >> 8 & 0xff) );	
	cmdState = PREFIX_2;
        return true;
      }
      else if( 'h' == cmdByte )
      {
        //
        // Am I alive? command
        //
		
        //
        // Send $ value to the PDA
	//
        //vUARTIOputsBT( BANNER );
        //xQueueSend( xSongQueue, ( void * ) &cmdByte, portMAX_DELAY  );
        //SerialPort.println("Hello");
	cmdState = PREFIX_2;
        return true;
      }	
      //Left
      else if( 'l' == cmdByte )							// Turn left
      {
        //vMotorDriveRadius( 1, SPEED_15, REMOTE_CONTROL );
        vMotorDriveRadius( 1, 50, 0, REMOTE_CONTROL );
        cmdState = PREFIX_2;
        return true;
      }
      //Right
      else if( 'r' == cmdByte )							// Turn right
      {
        //vMotorDriveRadius( -1, SPEED_15, REMOTE_CONTROL );
        vMotorDriveRadius( -1, 50, 0, REMOTE_CONTROL );
	cmdState = PREFIX_2;
        return true;
      }
      else if( 'S' == cmdByte )							// Both motors stop
      {
        vMotorDriveRequest( 0, 0, 0, REMOTE_CONTROL );
        //SerialPort.println("Stop");
	cmdState = PREFIX_2;
        return true;
      }
   break;

   default:
	cmdState = PREFIX_2;
        return true;
        //SerialPort.println("Default");
   break;
  }
  return false;
			//if( cmdState == CMD_READ )
			//{
				/*
			else if( cmdState == READ_SPEED_1 )
			{
				speedL = cByteRxed;
				cmdState = READ_SPEED_2;
			}
			else if( cmdState == READ_SPEED_2 )
			{
				speedR = cByteRxed;

				//Forward
				if( cmdByte == 'F' )							// Both motors forward
				{
					//speed = fgetc( CMD_PORT );				// Read the motor speed
					vMotorDriveRequest( speedL, speedR, REMOTE_CONTROL );	
				}
		
				//Reverse
				else if( cmdByte == 'R' )							// Both motors reverse
				{
					//speed = fgetc( CMD_PORT );				// Read the motor speed
		
					vMotorDriveRequest( -speedL, -speedR, REMOTE_CONTROL );	
				}

				cmdState = PREFIX_1;
			}
*/
//			else
//			{
//				cmdState = PREFIX_1;
//			}
//		}		
}

