/*

  Basic.ino
  Chris Chronopoulos, 2023

  Simple, single-core Quadrant example for instructive purposes.

*/

#include "Quadrant.h"
Quadrant quadrant;

int thresh;
float cv;

void setup() {

  quadrant.begin();
  thresh = quadrant.dsp.getEngagementThreshold();

}

void loop() {

  // take lidar measurement and update internal state variables
  quadrant.update();

  // set LED's based on whether each lidar is engaged (target within range)
  for (int i=0; i<4; i++) {
    if (quadrant.dsp.isLidarEngaged(i)) {
      quadrant.out.setLed(i, HIGH);
    } else {
      quadrant.out.setLed(i, LOW);
    }
  }

  // set the 4 CV channels based on the 4 (normalized) lidar distances

  cv = (5.0 * quadrant.dsp.getLidarDistance(0)) / thresh;
  quadrant.out.setCV(0, cv);

  cv = (5.0 * quadrant.dsp.getLidarDistance(1)) / thresh;
  quadrant.out.setCV(1, cv);

  cv = (5.0 * quadrant.dsp.getLidarDistance(2)) / thresh;
  quadrant.out.setCV(2, cv);

  cv = (5.0 * quadrant.dsp.getLidarDistance(3)) / thresh;
  quadrant.out.setCV(3, cv);

  // print a status report (in JSON format) to the USB serial monitor
  quadrant.out.printReportToSerial();

}

