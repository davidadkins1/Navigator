/*******************************************************************************
 * FileName:    Weblot.ino
 * Product:     ClipBoardBot
 * Processor:   Arduino AVR
 * Purpose:	Main setup and loop code
 *
 * (c)2013 David Adkins
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Revision History: 
 *
 * Revision 0.1 09/13 by David Adkins: 
 * 		1. Creation of code base
 ******************************************************************************/
//#include "iocfg.h"
#include <Servo.h>
#include "behaviors.h"

// the setup routine runs once when you press reset:
void setup () 
{
  motor_initialize();
  uartio_intialize();
  init_compass();
  navigation_init();
  sonar_initialize();
}

// the loop routine runs over and over again forever:
void loop () 
{
  vMotorMonitor();
  vSonarIO();
  vArbitrate();
  vUARTRxControl();
  compass_task();
  gps_task();
}


