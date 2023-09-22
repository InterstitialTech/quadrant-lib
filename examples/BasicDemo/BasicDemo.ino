#include "quadrant.h"

#define LED_THRESH 180

Quadrant myquad;

int values[4] = {8192, 8192, 8192, 8192};
char serial_buf[21];

void setup() {

  myquad.begin();
  myquad.ledsOff();
  myquad.setLidarsContinuous();

}

void loop() {

  int value;
  unsigned long t_last, t_now;
  bool newValue;

  t_last = micros();
  t_now = micros();

  newValue = false;
  for(int i=0; i<4; i++){
    if (myquad.checkLidarContinuous(i)) {
      value = myquad.readLidarContinuous(i);
      if (value < LED_THRESH) {
        digitalWrite(myquad.leds[i], HIGH);
      } else {
        digitalWrite(myquad.leds[i], LOW);
      }
      myquad.writeDac(i, value);
      values[i] = value;
      newValue = true;
    }
  }

  if (newValue) {
    sprintf(serial_buf, "%d %d %d %d", values[0], values[1], values[2], values[3]);
    Serial.println(serial_buf);
  }

}

