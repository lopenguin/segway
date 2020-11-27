#include "Arduino.h"
#include "sensors.h"
// ICM
#include "ICM_20948.h"
// Distance sensor
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include "SparkFun_VL53L1X.h"

/**********************************
MOTOR CLASS
***********************************/
void Motor::begin() {
  pinMode(m_enPin, OUTPUT);
  pinMode(m_in1Pin, OUTPUT);
  pinMode(m_in2Pin, OUTPUT);

  drive(0);
}

void Motor::drive(int s) {
  // check that -255<=s<=255
  if (s > 255) {
    Serial.println("Tried to go too fast. Writing to maximum speed");
    s = 255;
  }
  if (s < -255) {
    Serial.println("Tried to go too fast. Writing to maximum speed");
    s = -255;
  }

  // check if already set at this speed
  if ((m_speed == s || m_speed == -s) && s != 0)
    return;

  m_speed = s;

  if (s < 0) {
    s = -s;
    setDir(0);
  } else
    setDir(1);

  analogWrite(m_enPin,s);
}

void Motor::setDir(bool dir)
{
  if (dir)
  {
    digitalWrite(m_in1Pin, 0);
    digitalWrite(m_in2Pin, 1);
  }
  else
  {
    digitalWrite(m_in1Pin, 1);
    digitalWrite(m_in2Pin, 0);
  }
}

/**********************************
ICM Functions
***********************************/

/* Sets up ICM
 * Input: the ICM object
*/
void senBegin(ICM_20948_I2C &myI)
{
  bool initialized = false;
  // Make sure initialization occurs
  while(!initialized)
  {
    // Change the 1 to a 0 if the ADR jumper is closed.
    myI.begin(Wire, 1);
    Serial.print("IMU initialization returned: ");
    Serial.println(myI.statusString());
    if (myI.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
}
/* Gets the current direction (1 float) based on calibration data.
Calibrate with ICM_calibrate function! It is big cool.
Sample calibration data and const values (magX,magY):
 * N: -51.75, 39.3 -- scaled to 0
 * E: -25.5, 16.8 -- scaled to 0.5
 * S: -46.95, -6.45 -- scaled to -1/1
 * W: -73.35, 16.05 -- scaled to -0.5
 * refX0 = -49.35
 * refX1dif = 23.93
 * refY0 = 16.43
 * refY1dif = 22.87
We actually use the scaled angle (degrees)
Note: there is a discontinuity at S between -1 and 1.
*/
float getDir(ICM_20948_I2C &myI)
{
  // Normalize values based on unit circle
  const float refX0{ -49.35 };     // Zero for x
  const float refX1dif{ 23.93 };  // Difference between 0 and 1 for x
  const float refY0{ 16.43 };    // Zero for y
  const float refY1dif{ 22.87 };  // Dif between 0 and 1 for y

  float dirX = (myI.magX() - refX0) / refX1dif;
  float dirY = (myI.magY() - refY0) / refY1dif;

  // Angle in radians
  // Divide by pi to get multiple, add 1 to make it between 0 and 2.
  return atan2(dirX, dirY) / 3.1415;
}

// Reads ICM data (very simple, but mostly here for modularity)
// Returns success/failure
bool senRead(ICM_20948_I2C &myI)
{
  if (myI.dataReady())
  {
    myI.getAGMT();
    return 1;
  }
  return 0;
}

/* To access ICM data (scaled based on startup conditions)
myI.accX()
myI.accY()
myI.accZ()
myI.gyrZ()
myI.gyrY()
myI.gryZ()
myI.magX()
myI.magY()
myI.magZ()
myI.temp()
*/

/**********************************
Distance Sensor Functions
***********************************/

/* Overloaded distance setup
 * Input: distance sensor object
*/
void senBegin(SFEVL53L1X &dist)
{
  if (dist.begin() != 0)  // Good init returns 0
  {
    Serial.println(F("Distance sensor failed to begin. Freezing..."));
    while(1)
      ;
  }

  // We only really need short range distance
  dist.setTimingBudgetInMs(50);
  dist.setIntermeasurementPeriod(50);

  Serial.println(F("Distance sensor online."));
}
