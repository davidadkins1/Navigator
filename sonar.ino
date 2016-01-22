#include <Servo.h>

static unsigned int duration;
static unsigned int cm;
static Servo SonarServo;
enum SONAR_STATE {SONAR_READ, SONAR_DELAY};
static enum SONAR_STATE sonar_state;
static enum ESCAPE_STATES escapeState;
static unsigned long sonar_timer; 
static unsigned long turn_timer; 

#define SERVO_PIN   53

void sonar_initialize(void)
{
  SonarServo.attach(SERVO_PIN);
  SonarServo.write(90);

  cm = 0;

  SONAR_STATE sonar_state = SONAR_READ;
  ESCAPE_STATES escapeState = NORMAL_CRUISE;
}

/*******************************************************************************
 *    Function Name:	vSonarIO
 *    Return Value:	none
 *    Parameters:	none
 *    Description:	Sonar I/O
 * The PING))) returns a pulse width of 73.746 uS per inch. Since the
 * Javelin pulseIn() round-trip echo time is in 8.68 uS units, this is the
 * same as a one-way trip in 4.34 uS units. Dividing 73.746 by 4.34 we
 * get a time-per-inch conversion factor of 16.9922 (x 0.058851).																	*				
 ******************************************************************************/
void vSonarIO( void )
{
  switch(sonar_state)
  {
    case SONAR_READ:
      pinMode(pingPin, OUTPUT);
      digitalWrite(pingPin, LOW);
      delayMicroseconds(2);
      digitalWrite(pingPin, HIGH);
      delayMicroseconds(5);
      digitalWrite(pingPin, LOW);
      pinMode(pingPin, INPUT);
      duration = pulseIn(pingPin, HIGH);

      // convert the time into a distance
      // The speed of sound is 340 m/s or 29 microseconds per centimeter.
      // The ping travels out and back, so to find the distance of the
      // object we take half of the distance travelled.
      cm = duration / 29 / 2;
      sonar_timer = set_timer(200);
      sonar_state = SONAR_DELAY;
    break;

    case SONAR_DELAY:
      if(time_out(sonar_timer))
      {
        sonar_state = SONAR_READ;        
      }
      else
      {
        return;
      }    
    break;
    
    default:
    break;
  }
 
  if( escapeState == NORMAL_CRUISE )
  {
    if( cm < 10 )
    {
      SonarServo.write(60);
      escapeState = TURN_AWAY;
    }
  }
  else if( escapeState == STALL_STOP )
  {
    vMotorDriveRequest( 0, 0, 0, ESCAPE );			// Stop
    escapeState = TURN_AWAY;
  }
  else if( escapeState == BACKUP )
  {
    
    vMotorDriveRequest( -SPEED_25, -SPEED_25, 0, ESCAPE );	// Backup
    escapeState = TURN_AWAY;
  }
  else if( escapeState == TURN_AWAY )
  {
    vMotorDriveRequest( -127, 127, 0, ESCAPE );	// Spin a random time
    turn_timer = set_timer(2000);
    escapeState = WAIT_TURN;
  }
  else if(escapeState == WAIT_TURN)
  {
    if(time_out(turn_timer))
    {
      escapeState = CHECK_TURN;        
    }
  }
  else if(escapeState == CHECK_TURN)
  {
    if( cm > 10 )
    {
      SonarServo.write(90);
      escapeState = NORMAL_CRUISE;
      vReleaseControl( ESCAPE );		// Escape behavior termination
    }
    else
    {
      escapeState = TURN_AWAY;
    }
  }
}

unsigned int range(void)
{
  return cm;
}


