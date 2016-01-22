#include "TinyGPS.h"                 // Special version for 1.0
#include "waypoints.h"
#include "behaviors.h"
#include "Navigator.h"

#define GPS_PORT Serial1
#define GPS_POWER   6

#define GPS_ON  HIGH
#define GPS_OFF LOW

#define WAYPOINT_RADIUS 8
TinyGPS gps;
int waypoint_bearing;       // Bearing to the current waypoint (in degrees)
byte current_waypoint;      // This variable stores the actual waypoint we are trying to reach..
int waypoints;              // Total number of waypoints 
unsigned int waypoint_distance;   // Distance to the current waypoint (in meters)
int heading_correction;

void navigation_init(void) 
{
  pinMode(GPS_POWER, OUTPUT);
  //digitalWrite(GPS_POWER, GPS_OFF);
  //delay(20000);
  digitalWrite(GPS_POWER, GPS_ON);
  
  GPS_PORT.begin(4800);
  delay(100);
  GPS_PORT.println("$PSRF100,1,9600,8,1,0*0D"); //  command to switch SIRFIII to NMEA, 9600, 8, N, 1 
  delay(100);
  GPS_PORT.begin(9600);  // switch finally back to 9600 Baud

  current_waypoint = 0;
  waypoints = sizeof(wps) / sizeof(LONLAT); // calculate the number of waypoints
}

void gps_task()
{
  if(feedgps())
  {
    navigate();
  }
}

// Feed data as it becomes available 
bool feedgps()
{
  while (GPS_PORT.available()) 
  {
    if(gps.encode(GPS_PORT.read()))
    {
      return true;
    }
  }
  
  return false;
}

/*************************************************************************
 * //Function to calculate the course between two waypoints
 * //I'm using the real formulas--no lookup table fakes!
 *************************************************************************/
int get_gps_course(float flat1, float flon1, float flat2, float flon2)
{
  float calc;
  float bear_calc;

  float x = 69.1 * (flat2 - flat1); 
  float y = 69.1 * (flon2 - flon1) * cos(flat1 / DEGREE_CONVERSION);

  calc = atan2(y,x);

  bear_calc = degrees(calc);

  if(bear_calc <= 1)
  {
    bear_calc = 360 + bear_calc; 
  }
  return bear_calc;
}

/***************************************************************************/
//Computes heading the error, and choose the shortest way to reach the desired heading
/***************************************************************************/
int heading_error(int PID_set_Point, int PID_current_Point)
{
  float PID_error=0;  //Temporary variable
   
  if(fabs(PID_set_Point - PID_current_Point) > 180) 
  {
    if(PID_set_Point - PID_current_Point < -180)
    {
      PID_error = (PID_set_Point + 360) - PID_current_Point;
    }
    else
    {
      PID_error = (PID_set_Point - 360) - PID_current_Point;
    }
  }
  else
  {
    PID_error = PID_set_Point - PID_current_Point;
  }

  return PID_error;
}

/*************************************************************************
 * //Function to calculate the distance between two waypoints
 * //I'm using the real formulas
 *************************************************************************/
unsigned int get_gps_dist(float flat1, float flon1, float flat2, float flon2)
{

    float x = 69.1 * (flat2 - flat1); 
    float y = 69.1 * (flon2 - flon1) * cos(flat1 / DEGREE_CONVERSION);

    return (float)sqrt((float)(x * x) + (float)(y * y)) * 1609.344; 
}

// Get and process GPS data
void navigate()
{
  float xy[2];
  int zone;
  float latitude;
  float longitude;
  unsigned long fix_age;
  unsigned long date;
  unsigned long time;
  int right_speed;
  int left_speed;
  unsigned long ground_speed = gps.speed();
  int alt = gps.altitude();             //Altitude above sea 

  gps.get_datetime(&date, &time, &fix_age);
  gps.f_get_position(&latitude, &longitude, &fix_age);
  zone = floor(((longitude + 180.0) / 6) + 1);  
  zone = LatLonToUTMXY(DegToRad(latitude), DegToRad(longitude), zone, xy);
/*
  Serial.print(zone);

  if(latitude < 0)
  {
    Serial.print(" South");
  }
  else
  {
    Serial.print(" North");
  }

  Serial.print(", ");
  Serial.print(xy[1]);
  Serial.print(", ");
  Serial.println(xy[0]);
*/
  waypoint_bearing = get_gps_course(latitude, longitude, wps[current_waypoint].lat, wps[current_waypoint].lon);  //Calculating Bearing, this function is located in the GPS_Navigation tab.. 
  heading_correction = heading_error(waypoint_bearing, current_compass_heading());
  waypoint_distance = get_gps_dist(latitude, longitude, wps[current_waypoint].lat, wps[current_waypoint].lon); //Calculating Distance, this function is located in the GPS_Navigation tab.. 
  
  // Ensure that the autopilot will jump ONLY ONE waypoint
  if(waypoint_distance < WAYPOINT_RADIUS) //Checking if the waypoint distance is less than WP_RADIUS m, and check if the lock is open
  {
    current_waypoint++; //Switch the waypoint   
  } // end if wp_distance...
  
  if(current_waypoint >= waypoints)    // Check if we've passed all the waypoints, if yes release motor control
  {
    vReleaseControl(NAVIGATE);
  }
  else
  { 
    // Steer toward waypoint         
    if(heading_correction >= 5)
    {
      right_speed = 180;
      left_speed = 255;
    }
    else if(heading_correction <= -5)
    {
      right_speed = 255;
      left_speed = 180;
    }
    else
    {
      right_speed = 255;
      left_speed = 255;
    }
    
    vMotorDriveRequest(left_speed, right_speed, 0, NAVIGATE);
  }
  

    SerialPort.write(0x80);         
    SerialPort.print((int)current_compass_heading());
    SerialPort.print(",");
    SerialPort.print((int)waypoint_bearing);   
    SerialPort.print(",");
    SerialPort.print(heading_correction);   
    SerialPort.print(",");
    SerialPort.print(waypoint_distance);   
    SerialPort.print(",");
    SerialPort.print(time);
    SerialPort.print(",");
    SerialPort.print((int)left_speed);      
    SerialPort.print(",");
    SerialPort.print((int)right_speed);      
    SerialPort.print(",");
    SerialPort.print((int)current_waypoint);
    SerialPort.print(",");
    SerialPort.print(latitude, 6);
    SerialPort.print(",");
    SerialPort.print(longitude, 6);
    SerialPort.print(",");
    //ground_speed *= 18.0; // Scale miles/h to km/h * 10
    SerialPort.print((int)ground_speed);
    SerialPort.print(",");
    SerialPort.print(alt);
    SerialPort.print(",");
    SerialPort.print(range());
    SerialPort.print('\r');
}

