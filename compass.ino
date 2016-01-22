/*********************************************************************
 *
 *      Metal Detector Robot
 *
 *********************************************************************
 * FileName:        Compass.c
 * Processor:       Arduino Mega 2560 AVR
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * David Adkins       01/08/15    Original.
 ********************************************************************/

/** I N C L U D E S **********************************************************/
// Reference the I2C Library
#include <Wire.h>
// Reference the HMC5883L Compass Library
#include <HMC5883L.h>

// Store our compass as a variable.
HMC5883L compass;

/** Defines ********************************************************/
#define DECLINATION_CORRECTION .1902401

/** Global Variables ********************************************************/
float compass_heading;

/** Local Variables ********************************************************/
static unsigned long compass_timer;

/*-----------------------------------------------------------*/
#pragma code
void init_compass(void)
{
  Wire.begin();
  
  compass = HMC5883L(); //new instance of HMC5883L library
  setupHMC5883L(); //setup the HMC5883L
  compass_timer = set_timer(100);
}

void setupHMC5883L()
{
  //Setup the HMC5883L, and check for errors
  int error;  
  error = compass.SetScale(1.3); //Set the scale of the compass.
  if(error != 0) Serial.println(compass.GetErrorText(error)); //check if there is an error, and print if so

  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) Serial.println(compass.GetErrorText(error)); //check if there is an error, and print if so
}

void compass_task(void)
{
  if(time_out(compass_timer))
  {
    compass_timer = set_timer(66);
    compass_heading = getHeading();
  }
}
    
float getHeading()
{
  //Get the reading from the HMC5883L and calculate the heading
  MagnetometerScaled scaled = compass.ReadScaledAxis(); //scaled values from compass.
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  float declinationAngle = DECLINATION_CORRECTION;
  heading += declinationAngle * 2;
     
  // Compass is rotated 90 degrees
  heading -= PI/2;

  // Correct for when signs are reversed.
  if(heading < 0) heading += 2*PI;
  if(heading > 2*PI) heading -= 2*PI;
    
  return heading * RAD_TO_DEG; //radians to degrees
}

float current_compass_heading(void)
{
    return compass_heading;
}
/*end of module*/


