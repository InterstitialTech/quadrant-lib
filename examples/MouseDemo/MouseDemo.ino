/*

  MouseDemo.ino
  Chris Chronopoulos, 2023

  Use Quadrant as a computer mouse (HID device).

  With all lidars engaged, the "pitch" parameter will control Y motion, and the
  "roll" parameter will control X motion. Crossing into the low "elevation"
  threshold will click the mouse. Flattening your hand or hyperextending your
  fingers (negative "arc") will switch to mousewheel control.
  
  The thresholds for these modes are set by the #define params below.

*/

#define ELEVATION_THRESHOLD_CLICK 0.1
#define ARC_THRESHOLD_SCROLL 0.02

#include "Quadrant.h"
Quadrant quadrant;

#include "Mouse.h"

void setup() {

  quadrant.begin();
  quadrant.enableFilter(true);
  quadrant.run();

  Mouse.begin();

}

void loop() {

  if (quadrant.newDataReady()) {

		// update local state machine
		quadrant.update();

    // set indicator leds
		for (int i=0; i<4; i++) {
			if (quadrant.isLidarEngaged(i)) {
				quadrant.setLed(i, HIGH);
			} else {
				quadrant.setLed(i, LOW);
			}
		}

    // mouse control
    if (quadrant.isElevationEngaged()) {
      // either scroll or move (and possibly click)
      if (quadrant.getArc() > ARC_THRESHOLD_SCROLL) {
        Mouse.move(0, 0, (-1) * quadrant.getPitch() * 20);
      } else {
        Mouse.move(quadrant.getRoll() * 127, quadrant.getPitch() * 127, 0);
        if (quadrant.getElevation() < ELEVATION_THRESHOLD_CLICK) {
          Mouse.click();
        }
      }
    }

  }

}

