/*

  PitchAndRoll.ino
  Chris Chronopoulos, 2023

  This sketch computes the pitch (front-to-back tilt) and roll (side-to-side
  tilt) of your hand above the board, and maps them to CV channels 1 and 2.

*/

#include "quadrant.h"

#define LED_THRESH 300

Quadrant myquad;

int values[4] = {8192, 8192, 8192, 8192};
char serial_buf[21];
bool engaged[4] = {false};

bool pitchEngaged(void) {
    return engaged[1] && engaged[3];
}

float pitch(void) {
    return float(values[1] - values[3]) / LED_THRESH;
}

bool rollEngaged(void) {
    return engaged[2] && engaged[0];
}

float roll(void) {
    return float(values[2] - values[0]) / LED_THRESH;
}

void setup() {

  myquad.begin();
  myquad.ledsOff();
  myquad.setLidarsContinuous();

}

void loop() {

  int value;
  unsigned long t_last, t_now;
  bool newValue;
  int dacValue;

  t_last = micros();
  t_now = micros();

  newValue = false;
  for(int i=0; i<4; i++){
    if (myquad.checkLidarContinuous(i)) {
      value = myquad.readLidarContinuous(i);
      if (value < LED_THRESH) {
        digitalWrite(myquad.leds[i], HIGH);
        engaged[i] = true;
      } else {
        digitalWrite(myquad.leds[i], LOW);
        engaged[i] = false;
      }
      values[i] = value;
      newValue = true;
    }
  }

  if (newValue) {
    sprintf(serial_buf, "%d %d %d %d", values[0], values[1], values[2], values[3]);
    Serial.println(serial_buf);
    if (pitchEngaged()) {
      dacValue = 512 + 512 * pitch() * 4;
      myquad.writeDac(1, dacValue);
      Serial.print("pitch: ");
      Serial.println(dacValue);
    }
    if (rollEngaged()) {
      dacValue = 512 + 512 * roll() * 6;
      myquad.writeDac(2, dacValue);
      Serial.print("roll: ");
      Serial.println(dacValue);
    }
  }

}

