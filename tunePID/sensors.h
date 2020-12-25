/* Contains forward declarations for the following sensors:
  * ICM-20948 (IMU)
  * Distance sensor
  * Motor class
*/
#ifndef SENSORS_H
#define SENSORS_H

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
class Motor {
  int m_enPin{};
  int m_in1Pin{};
  int m_in2Pin{};

  int m_speed{0};
public:
  // Constructor
  Motor(int enPin, int in1Pin, int in2Pin)
    : m_enPin{ enPin }, m_in1Pin{ in1Pin }, m_in2Pin{ in2Pin }
  {/*does nothing*/}

  // Initializes motor
  void begin();

  // Writes a speed to the motor
  // -255 <= s <= 255
  void drive(int s);

private:
  // Allows easy changing of directions
  // 1 is forward, 0 is backwards
  void setDir(bool dir);
};

/**********************************
ICM Functions
***********************************/

// Sets up the ICM
void senBegin(ICM_20948_I2C &myI);

// Gets current magnetometer direction
float getDir(ICM_20948_I2C &myI);

// Reads ICM data
bool senRead(ICM_20948_I2C &myI);

#endif

/**********************************
Distance Sensor Functions
***********************************/

// Sets up the distance sensor
void senBegin(SFEVL53L1X &dist);
