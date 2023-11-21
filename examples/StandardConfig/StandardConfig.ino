/*

  StandardConfig.ino
  Chris Chronopoulos, 2023

  This sketch represents the standard, out-of-the-box configuration for
  Quadrant. It measures all 4 lidar channels, and then (if accordingly engaged)
  computes the following aggregate parameters:

    * elevation: average height
    * pitch: front-to-back tilt
    * roll: side-to-side tilt
    * arc: curvature/cupping

  These parameters are then mapped to CV channels 0,1,2,3. The sensitivity of
  the CV outputs to each parameter can be adjusted by changing the #define
  params CV_GAIN_xx below.

  The lidar values, the sample rate, and aggregate parameters are all
  printed to USB serial in JSON format.

*/

#define CV_GAIN_ELEVATION 1.0
#define CV_GAIN_PITCH  3.0
#define CV_GAIN_ROLL   4.0
#define CV_GAIN_ARC    10.0

#include "Quadrant.h"

Quadrant quadrant;


void setup() {

  quadrant.begin();
  quadrant.setBoxcarLength(5);
  Serial.begin(115200);

}


void loop() {

  // take lidar measurement and update state variables
  //quadrant.update();
  quadrant.update_boxcar();

  // set indicator leds
  for (int i=0; i<4; i++) {
    if (quadrant.isLidarEngaged(i)) {
      quadrant.setLed(i, HIGH);
    } else {
      quadrant.setLed(i, LOW);
    }
  }

  // elevation to CV 0
  if (quadrant.isElevationEngaged()) {
    quadrant.setCV(0, quadrant.getElevation() * 5.0 * CV_GAIN_ELEVATION);
  }

  // pitch to CV 1
  if (quadrant.isPitchEngaged()) {
    quadrant.setCV(1, 2.5 + quadrant.getPitch() * 2.5 * CV_GAIN_PITCH);
  }

  // roll to CV 2
  if (quadrant.isRollEngaged()) {
    quadrant.setCV(2, 2.5 + quadrant.getRoll() * 2.5 * CV_GAIN_ROLL);
  }

  // arc to CV 3
  if (quadrant.isArcEngaged()) {
    quadrant.setCV(3, 2.5 + quadrant.getArc() * 2.5 * CV_GAIN_ARC);
  }

  // print status report to USB serial
  quadrant.printReportToSerial();

}

