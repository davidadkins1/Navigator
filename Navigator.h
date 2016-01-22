#ifndef ARLO_H
#define ARLO_H

#define PI 3.141593
#define NINETY_DEGREES      1.570796
#define ONE_EIGHTY_DEGREES  3.141593
#define TWO_SEVENTY_DEGREES 4.712389
#define THREE_SIXTY_DEGREES 6.283185
#define DECLINATION_CORRECTION .1902401
#define DEGREE_CONVERSION 57.29578

// Pins
// Interrupt 0 is on pin 2
// Interrupt 1 is on pin 3
#define LEFT_HB25_CONTROL 4 
#define RIGHT_HB25_CONTROL 5

//#define GPS_POWER 6

// Test LED
#define TEST_LED 8

// * 1 = 100 percent duty cycle (Al)
#define SPEED_100	2000
// * 3 = 75 percent duty cycle (75uS)
#define SPEED_75	1875
// * 2 = 50 percent duty cycle (50uS)
#define SPEED_50	1750
// * 1 = 25 percent duty cycle (25uS)
#define SPEED_25	1625
// * 0 = 0 percent duty cycle (Always low)
#define SPEED_0	        1500

// Wheel circumference in mm * 1000. 6inch wheels * PI = 18.85"
#define METRIC_CIRCUMFERENCE 483805
#define METRIC_SPEED 1000
#define METRIC_WHEEL_BASE 390.0
// In mm/sec
#define MIN_METRIC_SPEED 200
#define MAX_METRIC_SPEED 500
#define RADIAN_CONVERSION 0.0174533
#define MM_PER_RADIAN 76.2

#define MM_PER_TICK 13439035
// 13.439 mm * 256
#define SCALED_MM_PER_TICK 3440
#define SERVO_ZERO 1500
#define MIN_SERVO 1000
#define MAX_SERVO 2000

// Port selection
// Test LED
#define TEST_LED 4

// if sending out debug information
//#define SERIAL_DEBUG

// Port selection

// if using Bluetooth
#define BLUETOOTH_SUPPORTED
#define BLUETOOTH_CONTROL

#ifdef BLUETOOTH_SUPPORTED
  #define BluetoothPort Serial2
  
  #ifdef BLUETOOTH_CONTROL
    #define SerialPort Serial2
  #else
    #define SerialPort Serial
    #define DebugPort Serial2
  #endif
#else
  #define SerialPort Serial
#endif

// Sonar
#define  pingPin 22

#endif

