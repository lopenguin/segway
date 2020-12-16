/* Central code file for the segway's controls.
More information will be added here someday.

Future:
 * could make angle offset auto-calibrate at startup
 * Use feed forward for motion
 * Motion profiling ?

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

void setup() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(LED_BUILTIN, INPUT);
  segway.begin();

  // to make sure we get ICM data
  delay(100);
}

void loop() {
  // double e{ 0 };
  // segway.getError(e);
  // Serial.println(e*0.4);
  segway.stabilize();
  //segway.drive(150);
  // delay(100);
}
