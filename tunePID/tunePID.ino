/* This is mainly an interface for simple tuning of the Segway's PID control.
Entering numbers into the serial monitor allows you to change the parameters.
You change one at a time.

Written by Lorenzo Shaikewitz
*/

// libraries and custom files
#include "segway.h"
#include "sensors.h"

const int enPinr{23};
const int in1Pinr{22};
const int in2Pinr{17};
const int enPinl{15};
const int in1Pinl{20};
const int in2Pinl{21};
Segway segway{ Motor{enPinl, in1Pinl, in2Pinl},
               Motor{enPinr, in1Pinr, in2Pinr} };

double kp{ 790 };

void setup() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(LED_BUILTIN, INPUT);
  segway.begin();

  // to make sure we get ICM data
  delay(100);
  Serial.println("Constant beginning at " + static_cast<String>(kp));
}

void loop() {
  // double e{ 0 };
  // segway.getError(e);
  // Serial.println(e*0.4);
  segway.stabilize(kp);
  //segway.drive(255);
  // delay(100);

  // code for reading the kp
  if (Serial.available()) {
    double temp = Serial.parseFloat();
    if (temp < 0.01)
      return;
    kp = temp;
    Serial.println("Constant updated to " + static_cast<String>(kp));
  }
}
