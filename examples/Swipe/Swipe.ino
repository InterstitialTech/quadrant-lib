/*

  Swipe.ino
  Chris Chronopoulos, 2023

*/

#define SAMPLE_PERIOD_US 23000

#include "Quadrant.h"
Quadrant quadrant;

#include "SwipeMachine.h"
SwipeMachine swiper;


unsigned long tnow, tlast;

void setup(void) {

  quadrant.daq.begin();

  tnow = micros();
  tlast = tnow;

}

void loop(void) {

  tnow = micros();

  if ((tnow - tlast) > SAMPLE_PERIOD_US) {
    quadrant.daq.update();
    quadrant.daq.pushToFifo();
    tlast = tnow;
  }

}

void setup1(void) {

  Serial.begin(115200);
  delay(100);

  quadrant.dsp.begin();

  quadrant.out.begin();
  quadrant.out.displayStartupLeds();

}

void loop1(void) {

  // update DSP module
  quadrant.dsp.updateFromFifo();

  // set indicator leds
  for (int i=0; i<4; i++) {
    if (quadrant.dsp.isLidarEngaged(i)) {
      quadrant.out.setLed(i, HIGH);
    } else {
      quadrant.out.setLed(i, LOW);
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

