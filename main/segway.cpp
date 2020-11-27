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

  m_lastICMtime = millis();
  m_lastSpeedAvg = millis();

  Serial.println("--------Segway is on--------");
  digitalWrite(LED_BUILTIN, HIGH);
}


double Segway::getError(double& e) {
  // Calibration constants
  static const double angleOffset{ 0 }; // whatever the reading is when upright

  // Get ICM data
  {
    // make sure it only happens once every 50 ms
    unsigned long t{ millis() };
    if (t - m_lastICMtime > 50) {
      m_lastICMtime = t;
      senRead(m_ICM);
    } else {
      return 0;
    }
  }

  // Convert accel data into an angle
  // accZ is up/down, accX is forward/backward
  double accAngle{ atan(m_ICM.accX() / m_ICM.accZ()) - angleOffset };

  // get gyrY data
  double gyrY{ m_ICM.gyrY() };

  // We want the response to be based on angle and gyro data
  // gyro: we care about small values != 0 (but not too small)
    // A backwards tilt is +250
  // acc: we care about larger deviations from our ideal angle
  // So we multiple gyro by something big, acc by something small

  // Serial.print(accAngle * 180 / 3.14);
  // Serial.print(" deg | ");
  // Serial.print(gyrY);
  // Serial.print(" dps | const: ");

  // I want an angle of +-pi/6 deg to = gyr of 250
  // If this constant is changed,
  float accConst{ 480 };

  // - gyrY is because gyro is neg when angle is pos
  e = (accAngle - speedToAngle()) * accConst - gyrY;
  return 1;
}

// we only get new data every 50 ms
void Segway::stabilize() {
  // give it a time average
  int numAvg{ 3 };  // multiple of 50 ms
  static int speedBuf{ 0 };
  int speed{ 0 };
  int minSpeed{ 150 };

  // Constants
  double kp{ 0.4 };
  double ki{ 0 };
  double kd{ 0 };

  double error{ 0 };
  if (!getError(error))
    // return if not gettting new data
    return;
  //Serial.println(error);
  // propotional term
  double result{ error * kp };
  // add integral term
  static double totError{ 0 };
  totError += error;
  result += totError * ki;
  // add derivative term
  result += (error - m_lastError) * kd;
  m_lastError = error;
  //Serial.println(result);

  // Convert into speed
  if (result > 0)
    // we are tilting forward, wanna spin wheels backwards
    speed = max(-255, -(minSpeed + result));
  else
    // we are tilting backward, wanna spin wheels forward
    speed = min( 255, -(-minSpeed + result));
  if (abs(speed) < minSpeed)
    speed = 0;
  // go!

  speedBuf += speed;
  unsigned long t{ millis() };
  if (t - m_lastSpeedAvg > numAvg * 50) {
    Serial.print("Speed: ");
    Serial.println(speedBuf / numAvg);
    drive(speedBuf / numAvg);
    speedBuf = 0;
    m_lastSpeedAvg = t;
  }
}


double Segway::speedToAngle(int s) {
  // temporary
  return s;
}


/* Note that wheels should move reverse when the car is tilting forward
*/
void Segway::drive(int s) {
  m_leftMotor.drive(s);
  m_rightMotor.drive(s);
}

/**********************************
PID HELPER FUNCTIONS
***********************************/
