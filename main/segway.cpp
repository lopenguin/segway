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
  //senBegin(m_distSen);

  m_lastICMtime = millis();
  m_lastSpeedAvg = millis();

  Serial.println("--------Segway is on--------");
  digitalWrite(LED_BUILTIN, HIGH);
}


double Segway::getError(double& e) {
  // Calibration constants
  static const double angleOffset{ -0.1 }; // whatever the reading is when upright

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

  // get gyrY data (temporarily removed)
  double gyrY{ m_ICM.gyrY() };

  // We want the response to be based on angle and gyro data
  // gyro: we care about small values != 0 (but not too small)
    // A backwards tilt is +250
  // acc: we care about larger deviations from our ideal angle
  // So we multiple gyro by something big, acc by something small

  // Serial.println(accAngle); //* 180 / 3.14);
  // Serial.print(" deg | ");
  // Serial.print(gyrY);
  // Serial.print(" dps | const: ");

  // I want an angle of +-pi/6 deg to = gyr of 250
  // If this constant is changed,
  float accConst{ 480 };

  // - gyrY is because gyro is neg when angle is pos
  e = (accAngle - speedToAngle()) * accConst - gyrY;
  // Serial.println(e);
  return 1;
}

// we only get new data every 50 ms
void Segway::stabilize() {
  // update the driving algorithm
  //drive();
  // give it a time average
  int numAvg{ 3 };  // multiple of 50 ms
  static int speedBuf{ 0 };
  int speed{ 0 };
  int minSpeed{ 180 };

  // Constants
  double kp{ 1.8 };
  double ki{ 0 };
  double kd{ 0 };

  double error{ 0 };
  if (!getError(error))
    // return if not gettting new data
    // this causes this function to be run once every 50 ms
    return;
  //Serial.println(error);
  // propotional term
  double result{ error * kp };
  // Serial.print("Error: ");
  // Serial.println(result);
  // add integral term
  static double totError{ 0 };
  totError += error;
  result += totError * ki;
  // add derivative term
  result += (error - m_lastError) * kd;
  m_lastError = error;
  // Serial.println(result);

  // convert into speed
  // Positive result means tilting forward, but we want wheels to spin backwards
  speed = -static_cast<int>(result);
  if (result > 255)
    speed = 255;
  else if (result < -255)
    speed = -255;

  speedBuf += speed;
  unsigned long t{ millis() };
  if (t - m_lastSpeedAvg > static_cast<unsigned int>(numAvg) * 50) {
    Serial.print("Speed: ");
    speedBuf /= numAvg;
    if (abs(speedBuf) < minSpeed)
      speedBuf = 0;
    Serial.println(speedBuf);
    m_lastPIDtime = t;
    drive(speedBuf);
    speedBuf = 0;
    m_lastSpeedAvg = t;
  }
}


double Segway::speedToAngle(int s) {
  // temporary
  return s;
}

/* Note that wheels should move reverse when the car is tilting forward.
This is used for temporary wheel changes; use setSpeed to change the segway's speed
*/
void Segway::drive(int s) {
  m_leftMotor.drive(s);
  m_rightMotor.drive(s);
}
void Segway::drive() {
  unsigned long t{ millis() };
  // stops PID correcting drive after certain number of ms
  if (t - m_lastPIDtime > 65) {
    drive(m_speed);

  }
}

/**********************************
PID HELPER FUNCTIONS
***********************************/
