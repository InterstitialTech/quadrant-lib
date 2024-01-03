/*

  NewBasic.ino
  Chris Chronopoulos, 2023

*/

#include "Quadrant.h"
Quadrant quadrant;

#define THRESH QUADRANT_THRESH_DEFAULT_MM

void setup() {

  quadrant.begin();
  quadrant.run();


}

void loop() {

  if (quadrant.newDataReady()) {

		// update local state machine and status report
		quadrant.update();

		// set LED's based on whether each lidar is engaged (target within range)
		for (int i=0; i<4; i++) {
			if (quadrant.isLidarEngaged(i)) {
				quadrant.setLed(i, HIGH);
			} else {
				quadrant.setLed(i, LOW);
			}
		}

		// set the 4 CV channels based on the 4 (normalized) lidar distances
		quadrant.setCV(0, (5.0 * quadrant.getLidarDistance(0)) / THRESH);
		quadrant.setCV(1, (5.0 * quadrant.getLidarDistance(1)) / THRESH);
		quadrant.setCV(2, (5.0 * quadrant.getLidarDistance(2)) / THRESH);
		quadrant.setCV(3, (5.0 * quadrant.getLidarDistance(3)) / THRESH);

		// print a status report (in JSON format) to the USB serial monitor
		quadrant.printReportToSerial();

  }

}

