/*

  Attitude.ino
  Chris Chronopoulos, 2023

  "Attitude" is the collective name for pitch, roll, and yaw (e.g. in aviation).

  This sketch computes the:
    * elevation: average height
    * pitch: front-to-back tilt)
    * roll: side-to-side tilt
    * arc: curvature, which can be used to calculate yaw
  of your hand, and maps them to CV channels 0,1,2,3.

*/

#include "quadrant.h"


Quadrant quadrant;

char serial_buf[21];


void setup() {

  quadrant.begin();
  quadrant.setLidarsContinuous();
  quadrant.initStateMachine();

}


void loop() {

  int dacValue;

  // update state machine
  quadrant.updateStateMachine();

  // print sample rate
  Serial.print("Sample Rate (Hz): ");
  Serial.println(quadrant.sampleRate());

  // print heights
  sprintf(serial_buf, "%d %d %d %d", quadrant.height(0), quadrant.height(1), quadrant.height(2), quadrant.height(3));
  Serial.println(serial_buf);

  // indicator leds
  for (int i=0; i<4; i++) {
    if (quadrant.engaged(i)) {
      quadrant.ledOn(i);
    } else {
      quadrant.ledOff(i);
    }
  }

  // elevation to CV 0
  if (quadrant.elevationEngaged()) {
    dacValue = quadrant.elevation() * 3.4;
    quadrant.writeDac(0, dacValue);
    Serial.print("elevation CV: ");
    Serial.println(dacValue);
  }

  // pitch to CV 1
  if (quadrant.pitchEngaged()) {
    dacValue = 512 + 512 * quadrant.pitch() * 4;
    quadrant.writeDac(1, dacValue);
    Serial.print("pitch CV: ");
    Serial.println(dacValue);
  }

  // roll to CV 2
  if (quadrant.rollEngaged()) {
    dacValue = 512 + 512 * quadrant.roll() * 6;
    quadrant.writeDac(2, dacValue);
    Serial.print("roll CV: ");
    Serial.println(dacValue);
  }

  // arc to CV 3
  if (quadrant.arcEngaged()) {
    dacValue = 512 + 512 * quadrant.arc() * 5;
    quadrant.writeDac(3, dacValue);
    Serial.print("arc CV: ");
    Serial.println(dacValue);
  }

}

