/* Central code file for the segway's controls.
More information will be added here someday.

Written by Lorenzo Shaikewitz
*/

// libraries and custom files
#include "segway.h"
#include "sensors.h"

const int enPinl{23};
const int in1Pinl{22};
const int in2Pinl{17};
const int enPinr{16};
const int in1Pinr{20};
const int in2Pinr{21};
Segway segway{ Motor{enPinl, in1Pinl, in2Pinl},
               Motor{enPinr, in1Pinr, in2Pinr} };

void setup() {
  Wire.begin();
  Serial.begin(115200);
  segway.begin();

  // to make sure we get ICM data
  delay(100);
}

void loop() {
  segway.getError();
  delay(100);
}
