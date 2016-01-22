/*******************************************************************************
 * FileName:    motor.ino
 * Product:	ClipBoardBot
 * Processor:   Arduino AVR
 * Purpose:	Motor control and arbitration. 
 *
 * (c)2013 David Adkins
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Revision History: 
 *
 * Revision 0.1 09/13 by David Adkins: 
 * 		1. Creation of code base
 ******************************************************************************/
#include "motor.h"
#include "behaviors.h"
#include "oi.h"
//#include "iocfg.h"

/** C O N F I G U R A T I O N *************************************************/

//#define MOTOR_NOISE 10
//#define MM_PER_COUNT 7.5
#define METRIC_CIRCUMFERENCE 175929
#define METRIC_SPEED 1000
#define METRIC_WHEEL_BASE (float)213
#define MAX_METRIC_SPEED 200
#define RADIAN_CONVERSION 0.0174533
#define MM_PER_RADIAN 130.5

/** Global Variables ********************************************************/


/** Local Variables ********************************************************/

static struct MOTOR_CONTROL_REQUESTS motorControl[MAX_BEHAVIOR];
static enum BEHAVIORS behaviorPriority[MAX_BEHAVIOR];

static xMotorControlParameters *LeftMotor;
static xMotorControlParameters *RightMotor;
static float b2r;
static int speedLeft;
static int speedRight;

static int vError;
static int abs_speed;
static unsigned long delta_time;
static unsigned long current_velocity;
static DWORD_VAL current_distance;
static DWORD_VAL distance_delta_accumulator;

/*-----------------------------------------------------------*/
static void motor_drive( struct MOTOR_CONTROL_REQUESTS *motor_parameters );
static void vMotorDriveLeft( int speed, unsigned long distance );
static void vMotorDriveRight( int speed, unsigned long distance );

static void LeftMotorSpeed(int speed)//
{
  static int left_speed = 0;
  
  if(left_speed != speed)
  {
    #ifdef SERIAL_DEBUG
    Serial.print(" Left Speed: ");
    Serial.println(speed);
    #endif

    analogWrite(LMS, speed);//input a simulation value to set the speed
    left_speed = speed;
  }
}

static void RightMotorSpeed(int speed)//
{
  static int right_speed = 0;
  
  if(right_speed != speed)
  {  
    #ifdef SERIAL_DEBUG
    Serial.print("Right Speed: ");
    Serial.print(speed);
    #endif

    analogWrite(RMS,speed);//input a simulation value to set the speed
    right_speed = speed;
  }
}

void motor_initialize( void )
{
  pinMode(RMR,OUTPUT);
  pinMode(RMF,OUTPUT);
  pinMode(RMS,OUTPUT);
  pinMode(LMR,OUTPUT);
  pinMode(LMF,OUTPUT);
  pinMode(LMS,OUTPUT);
  
  LeftMotorSpeed( SPEED_0 );
  RightMotorSpeed( SPEED_0 );

  // Behavior initialization
  behaviorPriority[0] = REMOTE_CONTROL;
  behaviorPriority[1] = ESCAPE;
  behaviorPriority[2] = NAVIGATE;
  behaviorPriority[3] = CRUISE;

  for( int i = NO_CONTROL_REQUEST; i < MAX_BEHAVIOR; i++ )
  {
    motorControl[i].BehaviorControl = NO_CONTROL_REQUEST;			// No behavior asking for control
    motorControl[i].left_motor.Speed = SPEED_0;
    motorControl[i].left_motor.Distance = 0;
    motorControl[i].right_motor.Speed = SPEED_0;
    motorControl[i].right_motor.Distance = 0;
  }

  // Create the structures used to pass parameters to the left motor task.
  LeftMotor = ( xMotorControlParameters * ) malloc( sizeof( xMotorControlParameters ) );
	
  if( LeftMotor != NULL )
  {
    LeftMotor->Select = LEFT_MOTOR;
    LeftMotor->State = STOPPED;
    LeftMotor->Speed = 0;
    LeftMotor->CurrentVelocity = 0;

    LeftMotor->Direction = LM_FORWARD_DISABLE;
    LeftMotor->CurrentDirection = LM_FORWARD_DISABLE;
    LeftMotor->Distance = 0;
  }
		
  // Create the structures used to pass parameters to the left motor task.
  RightMotor = (xMotorControlParameters *)malloc(sizeof( xMotorControlParameters));
	
  if( RightMotor != NULL )
  {
    RightMotor->Select = RIGHT_MOTOR;
    RightMotor->State = STOPPED;
    RightMotor->Speed = 0;
    RightMotor->CurrentVelocity = 0;
    
    #if 0
    RightMotor->MotorCurrent = 0;
    RightMotor->LastEMFTime = 0;
    RightMotor->EMFTime = 0;
    RightMotor->MotorEMF = 0;
    RightMotor->LastEMF = 0;
    RightMotor->MotorEMFB_channel = ADC_CH1;
    RightMotor->MotorEMFA = 0;
    RightMotor->MotorEMFB = 0;
    #endif
    
    RightMotor->Direction = RM_FORWARD_DISABLE;
    RightMotor->CurrentDirection = RM_FORWARD_DISABLE;
    RightMotor->Distance = 0;
  }	

  // Force Remote control
  vMotorDriveRequest( SPEED_0, SPEED_0, 0, REMOTE_CONTROL );
}

static void vArbitrate( void )
{
  enum BEHAVIORS winner = NO_CONTROL_REQUEST;
	
  // Step through the behaviors. Highest priority first
  for( int i = 0; i < MAX_BEHAVIOR; i++ )
  {
    // if this priority behavior requires control
    // then grant it.
    if( motorControl[behaviorPriority[i]].BehaviorControl != NO_CONTROL_REQUEST )
    {
      // motor control code goes here
      winner = motorControl[behaviorPriority[i]].BehaviorControl;
      motor_drive(&(motorControl[winner]));
      break;
    }
  }
}

// Called by arbtrator. Converts speed from mm/s to a motor PWM setting
static void motor_drive( struct MOTOR_CONTROL_REQUESTS *motor_parameters )
{
  //Serial.println(motor_parameters->right_motor.Speed);

  if( motor_parameters->right_motor.Speed >= MAX_METRIC_SPEED )
  {
    //vMotorDriveRight( SPEED_100, motor_parameters->right_motor.Distance );
    digitalWrite(RMF, RM_FORWARD_ENABLE);
    digitalWrite(RMR, RM_REVERSE_DISABLE);
    RightMotorSpeed(SPEED_100);
  }
  else if( motor_parameters->right_motor.Speed >= 150 )
  {
    //vMotorDriveRight( SPEED_75, motor_parameters->right_motor.Distance );
    digitalWrite(RMF, RM_FORWARD_ENABLE);
    digitalWrite(RMR, RM_REVERSE_DISABLE);
    RightMotorSpeed(SPEED_75);
  }
  else if( motor_parameters->right_motor.Speed >= 100 )
  {
    //vMotorDriveRight( SPEED_50, motor_parameters->right_motor.Distance );
    digitalWrite(RMF, RM_FORWARD_ENABLE);
    digitalWrite(RMR, RM_REVERSE_DISABLE);
    RightMotorSpeed(SPEED_50);
  }
  else if( motor_parameters->right_motor.Speed >= 50 )
  {
    //vMotorDriveRight( SPEED_25, motor_parameters->right_motor.Distance );
    digitalWrite(RMF, RM_FORWARD_ENABLE);
    digitalWrite(RMR, RM_REVERSE_DISABLE);
    RightMotorSpeed(SPEED_25);
  }
  else if( motor_parameters->right_motor.Speed >= 0 )
  {
    //vMotorDriveRight( SPEED_0, motor_parameters->right_motor.Distance );
    digitalWrite(RMF, RM_FORWARD_DISABLE);
    digitalWrite(RMR, RM_REVERSE_DISABLE);
    RightMotorSpeed(SPEED_0);
  }
  else if( motor_parameters->right_motor.Speed <= -MAX_METRIC_SPEED )
  {
    //vMotorDriveRight( SPEED_100, motor_parameters->right_motor.Distance );
    digitalWrite(RMF, RM_FORWARD_DISABLE);
    digitalWrite(RMR, RM_REVERSE_ENABLE);
    RightMotorSpeed(SPEED_100);
  }
  else if( motor_parameters->right_motor.Speed <= -150 )
  {
    //vMotorDriveRight( SPEED_75, motor_parameters->right_motor.Distance );
    digitalWrite(RMF, RM_FORWARD_DISABLE);
    digitalWrite(RMR, RM_REVERSE_ENABLE);
    RightMotorSpeed(SPEED_75);
  }
  else if( motor_parameters->right_motor.Speed <= -100 )
  {
    //vMotorDriveRight( SPEED_50, motor_parameters->right_motor.Distance );
    digitalWrite(RMF, RM_FORWARD_DISABLE);
    digitalWrite(RMR, RM_REVERSE_ENABLE);
    RightMotorSpeed(SPEED_50);
  }
  else if( motor_parameters->right_motor.Speed <= -50 )
  {
    //vMotorDriveRight( SPEED_25, motor_parameters->right_motor.Distance );
    digitalWrite(RMF, RM_FORWARD_DISABLE);
    digitalWrite(RMR, RM_REVERSE_ENABLE);
    RightMotorSpeed(SPEED_25);
  }
  else if( motor_parameters->right_motor.Speed <= 0 )
  {
    //vMotorDriveRight( SPEED_0, motor_parameters->right_motor.Distance );
    digitalWrite(RMF, RM_FORWARD_DISABLE);
    digitalWrite(RMR, RM_REVERSE_DISABLE);
    RightMotorSpeed(SPEED_0);
  }

  if( motor_parameters->left_motor.Speed >= MAX_METRIC_SPEED )
  {
    //vMotorDriveLeft( SPEED_100, motor_parameters->left_motor.Distance );
    digitalWrite(LMF, LM_FORWARD_ENABLE);
    digitalWrite(LMR, LM_REVERSE_DISABLE);
    LeftMotorSpeed(SPEED_100);
  }
  else if( motor_parameters->left_motor.Speed >= 150 )
  {
    //vMotorDriveLeft( SPEED_75, motor_parameters->left_motor.Distance );
    digitalWrite(LMF, LM_FORWARD_ENABLE);
    digitalWrite(LMR, LM_REVERSE_DISABLE);
    LeftMotorSpeed(SPEED_75);
  }
  else if( motor_parameters->left_motor.Speed >= 100 )
  {
    //vMotorDriveLeft( SPEED_50, motor_parameters->left_motor.Distance );
    digitalWrite(LMF, LM_FORWARD_ENABLE);
    digitalWrite(LMR, LM_REVERSE_DISABLE);
    LeftMotorSpeed(SPEED_50);
  }
  else if( motor_parameters->left_motor.Speed >= 50 )
  {
    //vMotorDriveLeft( SPEED_25, motor_parameters->left_motor.Distance );
    digitalWrite(LMF, LM_FORWARD_ENABLE);
    digitalWrite(LMR, LM_REVERSE_DISABLE);
    LeftMotorSpeed(SPEED_25);
  }
  else if( motor_parameters->left_motor.Speed >= 0 )
  {
    //vMotorDriveLeft( SPEED_0, motor_parameters->left_motor.Distance )    digitalWrite(LMF, LM_FORWARD_DISABLE);
    digitalWrite(LMF, LM_FORWARD_DISABLE);
    digitalWrite(LMR, LM_REVERSE_DISABLE);
    LeftMotorSpeed(SPEED_0);
  }
  else if( motor_parameters->left_motor.Speed <= -MAX_METRIC_SPEED )
  {
    //vMotorDriveLeft( SPEED_100, motor_parameters->left_motor.Distance );
    digitalWrite(LMF, LM_FORWARD_DISABLE);
    digitalWrite(LMR, LM_REVERSE_ENABLE);
    LeftMotorSpeed(SPEED_100);
  }
  else if( motor_parameters->left_motor.Speed <= -150 )
  {
    //vMotorDriveLeft( SPEED_75, motor_parameters->left_motor.Distance );
    digitalWrite(LMF, LM_FORWARD_DISABLE);
    digitalWrite(LMR, LM_REVERSE_ENABLE);
    LeftMotorSpeed(SPEED_75);
  }
  else if( motor_parameters->left_motor.Speed <= -100 )
  {
    //vMotorDriveLeft( SPEED_50, motor_parameters->left_motor.Distance );
    digitalWrite(LMF, LM_FORWARD_DISABLE);
    digitalWrite(LMR, LM_REVERSE_ENABLE);
    LeftMotorSpeed(SPEED_50);
  }
  else if( motor_parameters->left_motor.Speed <= -50 )
  {
    //vMotorDriveLeft( SPEED_25, motor_parameters->left_motor.Distance );
    digitalWrite(LMF, LM_FORWARD_DISABLE);
    digitalWrite(LMR, LM_REVERSE_ENABLE);
    LeftMotorSpeed(SPEED_25);
  }
  else if( motor_parameters->left_motor.Speed <= 0 )
  {
    //vMotorDriveLeft( SPEED_0, motor_parameters->left_motor.Distance )    digitalWrite(LMF, LM_FORWARD_DISABLE);
    digitalWrite(LMF, LM_FORWARD_DISABLE);
    digitalWrite(LMR, LM_REVERSE_DISABLE);
    LeftMotorSpeed(SPEED_0);
  }
/*
        odometer = motor_parameters->left_motor.Distance;
	if( motor_parameters->right_motor.Speed == 0 )
	{
		vMotorDriveRight( motor_parameters->right_motor.Speed, motor_parameters->right_motor.Distance );
	}
	//else if( motor_parameters->right_motor.Speed >= METRIC_SPEED )
	//{
	//	vMotorDriveRight( 1, motor_parameters->right_motor.Distance );
	//}
	else
	{
    	// speed is in mm/s
    	// each back EMF count = METRIC_TO_SPEED mm
    	// so back EMF target = speed / METRIC_TO_SPEED
    	// METRIC_TO_SPEED is scaled by 10 so the requested speed must be too
		vMotorDriveRight((motor_parameters->right_motor.Speed * 10) / METRIC_TO_SPEED, motor_parameters->right_motor.Distance );
		//vMotorDriveRight(36, motor_parameters->right_motor.Distance );		
	}
    
	if( motor_parameters->left_motor.Speed == 0 )
	{
		vMotorDriveLeft( motor_parameters->left_motor.Speed, motor_parameters->left_motor.Distance );
	}
	//else if( motor_parameters->left_motor.Speed >= METRIC_SPEED )
	//{
	//	vMotorDriveLeft( 1, motor_parameters->left_motor.Distance );
	//}-
	else
	{
    	// speed is in mm/s
    	// each back EMF count = METRIC_TO_SPEED mm
    	// so back EMF target = speed / METRIC_TO_SPEED
    	// METRIC_TO_SPEED is scaled by 10 so the requested speed must be too
		vMotorDriveLeft((motor_parameters->left_motor.Speed * 10) / METRIC_TO_SPEED, motor_parameters->left_motor.Distance);
		//vMotorDriveLeft(36, motor_parameters->left_motor.Distance);
	}
		
	if( 0 != odometer)
	{
    	if( LeftMotor->CurrentDistance >= odometer)
    	{
        	// Execute an immediate stop
        	vMotorDriveLeft(0,0);
        	vMotorDriveRight(0,0);
        	vMotorDriveRequest(0, 0, 0, REMOTE_CONTROL);
        	odometer = 0;
    	}
	}
*/
}

/********************************************************************
*    Function Name:	vMotorMonitor				    *
*    Return Value:	none					    *
*    Parameters:	data:                         		    *
*    Description:	Read motor current and speed. Run motor	    *
*			control Arbiter.	                    *				
********************************************************************/
void vMotorMonitor( void )
{
  enum MOTOR_STATE {MOTOR_START, POWER_ON_DELAY, MOTORS_STOPPED, MOTORS_RUNNING};
  static enum MOTOR_STATE motor_state = MOTOR_START;
  static unsigned long motor_timer;
     
  switch(motor_state)
  {
    case MOTOR_START:
      // Start the default CRUISE behavior after a two second delay
      motor_timer = set_timer(2000); 
      motor_state = POWER_ON_DELAY;
    break;
        
   case POWER_ON_DELAY:
    if(time_out(motor_timer))
    {
      motor_state = MOTORS_STOPPED;
    }      
    break;

    case MOTORS_STOPPED:
      //vMotorDriveRequest( -270, 270, 440, CRUISE );
      vMotorDriveRequest(128,128, 0, CRUISE );
      //vMotorDriveRadius( -1, 270, 0, REMOTE_CONTROL );
      motor_state = MOTORS_RUNNING;
    break;
        
    case MOTORS_RUNNING:
    break;
        
    default:
    break;
  }
}

static void vMotorDriveLeft( int speed, unsigned long distance )
{
    LeftMotor->Speed = speed;
    
    if(speed < 0)
    {
        abs_speed = -speed;
    }
    else
    {
        abs_speed = speed;
    }

  // For proportional control: Error = SP - PV (Set Point - Process Variable)
  // SP = requested speed
  // PV = Motor back EMF
  //vError = abs_speed - LeftMotor->MotorEMF;
  LeftMotor->CurrentVelocity += vError;
	
	// Now calculate the distance moved between T1 EMF and T2 EMF readings

    // convert the average EMF to the current velocity
    // METRIC_TO_SPEED is scaled by 10 so the current velocity must be too	
    //current_velocity = (avg_emf * METRIC_TO_SPEED) / 10;
	
	// To calculate distance traveled between T1 and T2
	// we need the delta time
	//delta_time = LeftMotor->EMFTime - LeftMotor->LastEMFTime;
    
    //LeftMotor->CurrentDistance += (((avg_emf * 7) + (avg_emf / 2)) * delta_time) / 1000;
    //LeftMotor->CurrentDistance += ((current_velocity * delta_time) / 1000);
    current_distance.Val = (current_velocity * delta_time) / 1000;
    
    LeftMotor->CurrentDistance += current_distance.Val;

	if( LeftMotor->CurrentVelocity < -SPEED_100 )
	{
		LeftMotor->CurrentVelocity = -SPEED_100;
	}
	else if(LeftMotor->CurrentVelocity > SPEED_100)
	{
		LeftMotor->CurrentVelocity = SPEED_100;
	}

    distance_delta_accumulator.Val = 0;

    // Protect from interrupts
    //Convert from big endian to little
    distance_delta_accumulator.byte.HB = sensorPacket.sensor.iDistanceTraveled.byte.LB;
    distance_delta_accumulator.byte.LB = sensorPacket.sensor.iDistanceTraveled.byte.HB;

	if( speed == 0 )
	{
		digitalWrite(LMF, LM_FORWARD_DISABLE);
		digitalWrite(LMR, LM_REVERSE_DISABLE);
                analogWrite(LMS, 0);
		LeftMotor->CurrentVelocity = 0;
		LeftMotor->CurrentDistance = 0;
		LeftMotor->State = STOPPED;
	}
	else if( speed > 0 )
	{
		digitalWrite(LMF, LM_FORWARD_ENABLE);
		digitalWrite(LMR, LM_REVERSE_DISABLE);
		
		distance_delta_accumulator.sVal += current_distance.sVal;
	}
	else
	{
		digitalWrite(LMF, LM_FORWARD_DISABLE);
		digitalWrite(LMR, LM_REVERSE_ENABLE);
		
		distance_delta_accumulator.sVal -= current_distance.sVal;
	}
	
	if(distance_delta_accumulator.sVal > 32767)
	{
    	distance_delta_accumulator.sVal = 32767;
	}
	else if(distance_delta_accumulator.sVal < -32768)
	{
    	distance_delta_accumulator.sVal = -32768;
	}
    
    //Convert from big endian to little
    sensorPacket.sensor.iDistanceTraveled.byte.LB = distance_delta_accumulator.byte.HB;
    sensorPacket.sensor.iDistanceTraveled.byte.HB = distance_delta_accumulator.byte.LB;

	if( LeftMotor->CurrentVelocity < 0 )
	{
		LeftMotorSpeed(-LeftMotor->CurrentVelocity);
                //LeftMotorSpeed(128);
	}
	else if( LeftMotor->CurrentVelocity >= 0 )
	{
		LeftMotorSpeed(LeftMotor->CurrentVelocity);
                //LeftMotorSpeed(128);
	}
}

static void vMotorDriveRight( int speed, unsigned long distance )
{
  RightMotor->Speed = speed;

  // Save the current back EMF as last (T1 EMF)
  //RightMotor->LastEMF = RightMotor->MotorEMF;
    
  if(speed < 0)
  {
    abs_speed = -speed;
  }
  else
  {
    abs_speed = speed;
  }

  // For proportional control Error = SP - PV or Set Point - Process Variable
  // SP = requested speed
  // PV = Motor back EMF
  //vError = abs_speed - RightMotor->MotorEMF;
  //RightMotor->CurrentVelocity += vError;

  // Now calculate the distance moved between T1 EMF and T2 EMF readings
/*
	if(0 == RightMotor->LastEMFTime)
	{
    	RightMotor->LastEMFTime = RightMotor->EMFTime;
	}
	
	avg_emf = (RightMotor->LastEMF + RightMotor->MotorEMF) / 2;
	
	if(avg_emf < 0)
	{
    	avg_emf = -avg_emf;
	}

    // convert the average EMF to the current velocity
    // METRIC_TO_SPEED is scaled by 10 so the current velocity must be too	
	current_velocity = (avg_emf * METRIC_TO_SPEED) / 10;
	
	// To calculate distance traveled between T1 and T2
	// we need the delta time
	delta_time = RightMotor->EMFTime - RightMotor->LastEMFTime;
    
    //LeftMotor->CurrentDistance += (((avg_emf * 7) + (avg_emf / 2)) * delta_time) / 1000;
    RightMotor->CurrentDistance += ((current_velocity * delta_time) / 1000);
*/

	if( RightMotor->CurrentVelocity < -SPEED_100 )
	{
		RightMotor->CurrentVelocity = -SPEED_100;
	}
	else if( RightMotor->CurrentVelocity > SPEED_100 ) 
	{
		RightMotor->CurrentVelocity = SPEED_100;
	}

	if( speed == 0 )
	{
		digitalWrite(RMF, RM_FORWARD_DISABLE);
		digitalWrite(RMR, RM_REVERSE_DISABLE);
                analogWrite(RMS, 0);
		RightMotor->CurrentVelocity = 0;
		RightMotor->CurrentDistance = 0;
		RightMotor->State = STOPPED;
	}
	else if( speed > 0 )
	{
		digitalWrite(RMF, RM_FORWARD_ENABLE);
                digitalWrite(RMR, RM_REVERSE_DISABLE);
	}
	else
	{
		digitalWrite(RMF, RM_FORWARD_DISABLE);
		digitalWrite(RMR, RM_REVERSE_ENABLE);
	}

	if( RightMotor->CurrentVelocity < 0 )
	{
		RightMotorSpeed( -RightMotor->CurrentVelocity );
                //RightMotorSpeed( 128 );
	}
	else if( RightMotor->CurrentVelocity >= 0 )
	{
		RightMotorSpeed( RightMotor->CurrentVelocity );
                //RightMotorSpeed( 128 );
	}
}

// Speed is in mm/sec
void vMotorDriveRequest( int leftSpeed, int rightSpeed, long distance, enum BEHAVIORS behaviorID )
{
  //Serial.println(behaviorID);
	//velocityLeft[behaviorID] = leftSpeed;
	//velocityRight[behaviorID] = rightSpeed;
	//behaviorControl[behaviorID] = behaviorID;
  
	if( leftSpeed > MAX_METRIC_SPEED )
	{
		leftSpeed = MAX_METRIC_SPEED;
	}
	else if( leftSpeed < -MAX_METRIC_SPEED )
	{
		leftSpeed = -MAX_METRIC_SPEED;
	}
		
	if( rightSpeed > MAX_METRIC_SPEED )
  {
		rightSpeed = MAX_METRIC_SPEED;
  }
	else if( rightSpeed < -MAX_METRIC_SPEED )
	{
		rightSpeed = -MAX_METRIC_SPEED;
	}
		
	motorControl[behaviorID].left_motor.Speed = leftSpeed;
	motorControl[behaviorID].left_motor.Distance = distance;
	motorControl[behaviorID].right_motor.Speed = rightSpeed;
	motorControl[behaviorID].right_motor.Distance = distance;
	motorControl[behaviorID].BehaviorControl = behaviorID;
  //odometer = distance;
}

// radius:
//   0      = Straight, also allowed are 32767 and -32768 (0x7fff and 0x8000)
//   1      = Spin counter clockwise if positive spped
//  -1	    = Spin clockwise if positive speed
//            negative speed is reverse, positive speed is forward
void vMotorDriveRadius( int radius, int speed, unsigned long distance, enum BEHAVIORS behaviorID )
{
	//int speedLeft, speedRight;
	//float b2r;
  Serial.print(speed);
  Serial.print(", ");
	Serial.println(radius);
	//if( (radius == 0) || (radius == (int)32767) || (radius == (int)-32768) )
  if((radius == 0) || (radius == (int)32767))
	{
		vMotorDriveRequest( speed, speed, distance, behaviorID );
	}
	else if( radius == -1 )
	{
		vMotorDriveRequest( speed, -speed, distance, behaviorID );
	}
	else if( radius == 1 )
	{
		vMotorDriveRequest( -speed, speed, distance, behaviorID );
	}
	else
	{
		b2r = METRIC_WHEEL_BASE / (2 * radius);
		speedLeft = speed * (1 - b2r);
		speedRight = speed * (1 + b2r);
		vMotorDriveRequest( speedLeft, speedRight, distance, behaviorID );
	}
}

/********************************************************************
*    Function Name:	vMotorEscape									*
*    Return Value:	none											*
*    Parameters:	data:                         					*
*    Description:	Monitor motor current and speed.				*
*					Backup if current too high.						*				
********************************************************************/
/*
void vMotorEscape(void *pvParameters )
{
	for( ;; )
	{
		if( RightMotor->MotorCurrent > 600 )
		{
			if( digitalRead(RMF) == RM_FORWARD_ENABLE )				// Determine stall direction
			{
				leftEscapeSpeed = -40;
				rightEscapeSpeed = -20;
			}
			else
			{
				leftEscapeSpeed = 27;
				rightEscapeSpeed = 27;
			}

			escapeState = STALL;
		}
		//else if( (leftMotorCurrent > (currentVelocityLeft * 3)) || (leftMotorCurrent > 450) )
		else if( LeftMotor->MotorCurrent > 600 )
		{
			if( digitalRead(RMF) == RM_FORWARD_ENABLE )				// Determine stall direction
			{
				leftEscapeSpeed = -20;
				rightEscapeSpeed = -40;
			}
			else
			{
				leftEscapeSpeed = 27;
				rightEscapeSpeed = 27;
			}

			escapeState = STALL;
		}

		if( escapeState == STALL )
		{
			vMotorDriveRequest( 0, 0, 0, ESCAPE );			// Stop
			escapeState = STALL_STOP;
		}
		else if( escapeState == STALL_STOP )
		{
			if( (digitalRead(RMF) == digitalRead(RMR)) && (digitalRead(LMF) == digitalRead(LMR)) )			// Wait for stop
			{
				escapeState = MOVE_AWAY;
				vMotorDriveRequest( leftEscapeSpeed, rightEscapeSpeed, 0, ESCAPE );	// Move and turn away
			}
		}
		else if( escapeState == MOVE_AWAY )
		{
			if( (digitalRead(RMF) != digitalRead(RMR)) && (digitalRead(LMF) != digitalRead(LMR)) )			// Wait for go
			{
				//vTaskDelay( (portTickType) 2500 );		// Go for 2 seconds
				vMotorDriveRequest( -rightEscapeSpeed, -leftEscapeSpeed, 0, ESCAPE );	// Drive foward and turn more
				escapeState = TURN_FORWARD;
			}
		}
		else if( escapeState == TURN_FORWARD )
		{
    		motorControl[ESCAPE].BehaviorControl = NO_CONTROL_REQUEST;
			//behaviorControl[ESCAPE] = NO_CONTROL_REQUEST;	// Escape behavior termination
			escapeState = NORMAL_CRUISE;
		}

		//vTaskDelay( (portTickType) 1000 );				// Multi-task for one second
	}
}
*/

/*
void vMotorTurnHeading( int radius, int speed, unsigned int new_heading, enum BEHAVIORS behaviorID )
{
	int new_radius = 0;
	unsigned int degrees_turn;
	float radians_turn;
	unsigned long distance;
	
	unsigned int heading = current_heading();
	
	if(heading < new_heading)
	{
    	if((new_heading - heading) > 180)
    	{
        	new_radius = (int)1;
        	degrees_turn = (heading + 360) - new_heading; 
    	}
    	else
    	{
        	new_radius = (int)-1;
        	degrees_turn = new_heading - heading; 
    	}
	}
	else if(heading > new_heading)
	{
    	if((heading - new_heading) > 180)
    	{
        	new_radius = (int)-1;
        	degrees_turn = (new_heading + 360) - heading; 
    	}
    	else
    	{
        	new_radius = (int)1;
        	degrees_turn = heading - new_heading; 
    	}
	}
	else if(heading == new_heading)
	{
    	return;
	}
	
   	radians_turn = (float)degrees_turn * RADIAN_CONVERSION;
   	
	// radius = 0 is just a simple spin
    if(radius == 0)
    {
        radius = new_radius;
    	distance = radians_turn * MM_PER_RADIAN;
    }	
	
	if( radius == (int)-1 )
	{
		vMotorDriveRequest( speed, -speed, distance, behaviorID );
	}
	else if( radius == (int)1 )
	{
		vMotorDriveRequest( -speed, speed, distance, behaviorID );
	}
	else
	{
		b2r = METRIC_WHEEL_BASE / (2 * radius);
		speedLeft = speed * (1 - b2r);
		speedRight = speed * (1 + b2r);
		distance = radians_turn * radius;
		vMotorDriveRequest( speedLeft, speedRight, distance, behaviorID );
	}
}
*/

void stop()//
{
  #ifdef SERIAL_DEBUG
  Serial.println("All stop");
  #endif
  digitalWrite(RMS,LOW);// Unenble the pin, to stop the motor. this should be done to avid damaging the motor. 
  digitalWrite(LMS,LOW);
}

void vReleaseControl( enum BEHAVIORS behaviorID )
{
  motorControl[behaviorID].BehaviorControl = NO_CONTROL_REQUEST;
}

/*-----------------------------------------------------------*/


