#ifndef MOTOR_H
#define MOTOR_H
#include "behaviors.h"

void vMotorInitialize( void );
void vMotorDrive( int leftSpeed, int rightSpeed );
void vMotorDriveRequest( int leftSpeed, int rightSpeed, long distance, enum BEHAVIORS behaviorID );
void vMotorDriveRadius( int radius, int speed, unsigned long distance, enum BEHAVIORS behaviorID );
void vMotorTurnHeading( int radius, int speed, unsigned int new_heading, enum BEHAVIORS behaviorID );
void vReleaseControl( enum BEHAVIORS behaviorID );
//unsigned int current_heading(void);

// Motor control pins
//#define  SPEED_SENSOR_RIGHT  2    // Hall Effect switch on interrupt 0
//#define  SPEED_SENSOR_LEFT   3    // Hall Effect switch on interrupt 1

#define LMF  11   // Right motor foward enable to L298.
      // 1 = Left motor forward enabled, 0 = Left motor forward disabled.
#define LMR  8    // Right motor reverse enable to L298.
      // 1 = Left motor reverse enabled, 0 = Left motor reverse disabled.
#define LMS  9    // Right motor speed to L298 enable.
      // 0 = stop, 1 = Full speed, or 20 khz PWM
#define RMF  13   // Left motor foward enable to L298.
      // 1 = Right motor forward enabled, 0 = Right motor forward disabled.
#define RMR  12   // Left motor reverse enable to L298.
      // 1 = Right motor reverse enabled, 0 = Right motor reverse disabled.
#define RMS  10   // Left motor speed to L298 enable.
      // 0 = Stop, 1 = Full speed, or PWM

//** M O T O R S ***********************************************************/
#define RM_STOP 0
#define RM_START 1
#define RM_FORWARD_ENABLE   1
#define RM_FORWARD_DISABLE  0
#define RM_REVERSE_ENABLE   1
#define RM_REVERSE_DISABLE  0

#define LM_STOP 0
#define LM_START 1
#define LM_FORWARD_ENABLE   0
#define LM_FORWARD_DISABLE  1
#define LM_REVERSE_ENABLE   0
#define LM_REVERSE_DISABLE  1

#define SPEED_100	255					// * 1 = 100 percent duty cycle (Always high)
#define SPEED_75	192					// * 3 = 75 percent duty cycle (75uS)
#define SPEED_50	128					// * 2 = 50 percent duty cycle (50uS)
#define SPEED_25	64					// * 1 = 25 percent duty cycle (25uS)
#define SPEED_0		0					// * 0 = 0 percent duty cycle (Always low)

enum MOTOR_SELECT
{
  LEFT_MOTOR,
  RIGHT_MOTOR
};

enum MOTOR_STATE
{
  STOPPED,
  RUNNING,
  COMPLETED
};

typedef struct MOTOR_CONTROL_PARAMETERS
{
  enum MOTOR_SELECT 	  Select;
  enum MOTOR_STATE	    State;
  int16_t		            Speed;
  uint8_t		            Direction;
  uint32_t              Distance;
  int16_t               CurrentVelocity;
  uint8_t               CurrentDirection;
  uint32_t              CurrentDistance;
}xMotorControlParameters;

struct MOTOR_CONTROL_REQUESTS
{
  enum BEHAVIORS BehaviorControl;
  MOTOR_CONTROL_PARAMETERS left_motor;
  MOTOR_CONTROL_PARAMETERS right_motor;
};
#endif

