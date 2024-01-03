/*

  Swipe.ino
  Chris Chronopoulos, 2023

*/

#include "Quadrant.h"
Quadrant quadrant;

#include "SwipeMachine.h"
SwipeMachine swiper;

void setup(void) {

  quadrant.begin();
  quadrant.run();

}

void loop(void) {

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

    swiper.update(&quadrant.dsp);
    if (swiper.swipedLeft()) {
      Serial.println("swiped left!");
    }
    if (swiper.swipedRight()) {
      Serial.println("swiped right!");
    }

  }

}

