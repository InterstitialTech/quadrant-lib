/*

  StandardConfig.ino
  Chris Chronopoulos, 2023

  This sketch demonstrates the basic functionality of the board. It measures
  all four channels and maps them to the 4 CV outputs. It also prints the
  channel values over USB serial.

*/

#include "quadrant.h"

Quadrant quadrant;

char serial_buf[21];

void setup() {

  quadrant.begin();

}

void loop() {

  // update state machine
  quadrant.update();

  // print sample rate
  Serial.print("Sample Rate (Hz): ");
  Serial.println(quadrant.getSampleRate());

  // print lidar distances
  sprintf(serial_buf, "%d %d %d %d", quadrant.getLidarDistance(0), quadrant.getLidarDistance(1),
          quadrant.getLidarDistance(2), quadrant.getLidarDistance(3));
  Serial.println(serial_buf);

  // indicator leds
  for (int i=0; i<4; i++) {
    if (quadrant.isLidarEngaged(i)) {
      quadrant.setLed(i, HIGH);
    } else {
      quadrant.setLed(i, LOW);
    }
  }

  float cv;

  // elevation to CV 0
  if (quadrant.isElevationEngaged()) {
    cv = 2.5 + quadrant.getElevation();
    quadrant.setCV(0, cv);
    Serial.print("elevation CV: ");
    Serial.println(cv);
  }

  // pitch to CV 1
  if (quadrant.isPitchEngaged()) {
    cv = 2.5 + quadrant.getPitch();
    quadrant.setCV(1, cv);
    Serial.print("pitch CV: ");
    Serial.println(cv);
  }

  // roll to CV 2
  if (quadrant.isRollEngaged()) {
    cv = 2.5 + quadrant.getRoll();
    quadrant.setCV(2, cv);
    Serial.print("roll CV: ");
    Serial.println(cv);
  }

  // arc to CV 3
  if (quadrant.isArcEngaged()) {
    cv = 2.5 + quadrant.getArc();
    quadrant.setCV(3, cv);
    Serial.print("arc CV: ");
    Serial.println(cv);
  }

}

