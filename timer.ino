/*******************************************************************************
 * FileName:    timer.ino
 * Product:	ClipBoardBot
 * Processor:   Arduino AVR
 * Purpose:	Routines to support elapsed time variables
 *
 * (c)2013 David Adkins
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Revision History: 
 *
 * Revision 0.1 09/13 by David Adkins: 
 * 		1. Creation of code base
 ******************************************************************************/
unsigned long set_timer (unsigned long delay_in)
{
  return delay_in + millis();
}

boolean time_out(unsigned long delay_in)
{
  boolean time_elapsed = false;
  
    if(millis() >= delay_in)
    {
      time_elapsed = true;
    }

  return time_elapsed;
}

