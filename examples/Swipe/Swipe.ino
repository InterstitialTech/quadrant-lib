/*

  Swipe.ino
  Chris Chronopoulos, 2023

*/

#define SAMPLE_PERIOD_US 23000

#define LEFT_CHAN 0
#define UP_CHAN 1
#define RIGHT_CHAN 2
#define DOWN_CHAN 3

#include "Quadrant.h"
Quadrant quadrant;


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
  //quadrant.dsp.initFilter(4);

  quadrant.out.begin();
  quadrant.out.displayStartupLeds();

}

bool wasEngaged[4] = {false};
bool primeLeft = false;
bool primeUp = false;
bool primeRight = false;
bool primeDown = false;

void loop1(void) {

  // update DSP module
  quadrant.dsp.updateFromFifo();

  for (int i=0; i<4; i++) {

    // set indicator leds
    if (quadrant.dsp.isLidarEngaged(i)) {
      quadrant.out.setLed(i, HIGH);
    } else {
      quadrant.out.setLed(i, LOW);
    }

    // swipe logic
    if (!wasEngaged[i] && quadrant.dsp.isLidarEngaged(i)) { // rising edge
      if (i==LEFT_CHAN && wasEngaged[RIGHT_CHAN]) {
        primeLeft = true;
      } else if (i==RIGHT_CHAN && wasEngaged[LEFT_CHAN]) {
        primeRight = true;
      } else if (i==DOWN_CHAN && wasEngaged[UP_CHAN]) {
        primeDown = true;
      } else if (i==UP_CHAN && wasEngaged[DOWN_CHAN]) {
        primeUp = true;
      }
      wasEngaged[i] = true;
    } else if (wasEngaged[i] && !quadrant.dsp.isLidarEngaged(i)) {  // falling edge
      if (i==LEFT_CHAN && wasEngaged[RIGHT_CHAN]) {
        if (primeRight) {
          Serial.println("right swipe");
          primeRight= false;
        } else if (primeLeft) {
          // left fakeout
          primeLeft = false;
        }
      } else if (i==RIGHT_CHAN && wasEngaged[LEFT_CHAN]) {
        if (primeLeft) {
          Serial.println("left swipe");
          primeLeft = false;
        } else if (primeRight) {
          // right fakeout
          primeRight = false;
        }
      } else if (i==DOWN_CHAN && wasEngaged[UP_CHAN]) {
        if (primeUp) {
          Serial.println("up swipe");
          primeUp = false;
        } else if (primeDown) {
          // down fakeout
          primeDown = false;
        }
      } else if (i==UP_CHAN && wasEngaged[DOWN_CHAN]) {
        if (primeDown) {
          Serial.println("down swipe");
          primeDown = false;
        } else if (primeUp) {
          // up fakeout
          primeUp = false;
        }
      }
      wasEngaged[i] = false;
    }

  }

  /*
  // print status report to USB serial
  quadrant.out.updateReport(&quadrant.dsp);
  quadrant.out.printReportToSerial();
  */

}

