/* Contains forward declarations and class stuff for the segway

*/
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
SEGWAY CLASS
***********************************/
class Segway {
  // Motors
  Motor m_leftMotor;
  Motor m_rightMotor;

  // Sensors
  ICM_20948_I2C m_ICM{};
  SFEVL53L1X m_distSen{};

  int m_speed{0};
  unsigned long m_lastICMtime{0};
  unsigned long m_lastSpeedAvg{0};
  unsigned long m_lastPIDtime{0};

  double m_lastError{ 0 };
public:
  Segway(Motor l, Motor r) : m_leftMotor{l}, m_rightMotor{r}
    {/*does nothing*/}

  void begin();

  /* Uses accel data to compute angle and gyro data to determine angular speed
  theta = atan(accZ / accX)
  theta-dot = gyrY
  */
  double getError(double& e);

  /* does the high level control */
  void stabilize(double kp);

private:
  /* Accepts -255 <= s <= 255 */
  double speedToAngle(int s);
  double speedToAngle() { return speedToAngle(m_speed); }

public:
  void drive(int s);
  void drive();
};
