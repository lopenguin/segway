#include "Arduino.h"
#include "sensors.h"
#include "segway.h"
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
SEGWAY CLASS
***********************************/
void Segway::begin() {
  m_leftMotor.begin();
  m_rightMotor.begin();
  senBegin(m_ICM);
  senBegin(m_distSen);

  lastICMtime = millis();

  Serial.println("--------Segway is on--------");
}

/* Note that wheels should move reverse when the car is tilting forward
*/
void Segway::drive(int s) {

}

double Segway::getError() {
  // Get ICM data
  {
    // make sure it only happens once every 50 ms
    unsigned long t{ millis() };
    if (t - lastICMtime > 50) {
      lastICMtime = t;
      senRead(m_ICM);
    }
  }

  // Convert accel data into an angle
  // accZ is up/down, accX is forward/backward
  double accAngle{ atan(m_ICM.accZ() / m_ICM.accX()) };

  // get gyrY data
  double gyrY{ m_ICM.gyrY() };

  // We want the response to be based on angle and gyro data
  // gyro: we care about small values != 0 (but not too small)
  // acc: we care about larger deviations from our ideal angle
  // So we multiple gyro by something big, acc by something small
  float accConst{ 0.2 };

  Serial.print(accAngle);
  Serial.print(" radians | ");
  Serial.println(gyrY);

  return (accAngle - speedToAngle()) * accConst + gyrY * (1 - accConst);
}

double Segway::speedToAngle(int s) {
  // temporary
  return s;
}

/**********************************
PID HELPER FUNCTIONS
***********************************/
