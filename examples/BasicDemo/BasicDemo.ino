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

  // slower (~32 Hz) but synchronous
  /*
  if (myquad.checkLidarsContinuous()) {
    for(int i=0; i<4; i++){
      value = myquad.readLidarContinuous(i);
      if (value < LED_THRESH) {
        digitalWrite(myquad.leds[i], HIGH);
      } else {
        digitalWrite(myquad.leds[i], LOW);
      }
      myquad.writeDac(i, value);
      values[i] = value;
    }
    sprintf(serial_buf, "%d %d %d %d", values[0], values[1], values[2], values[3]);
    Serial.println(serial_buf);
    t_last = t_now;
    t_now = micros();
    Serial.print("Sample rate = ");
    Serial.println(1000./(t_now - t_last));
  }
  */

  // faster (~40-100 Hz) but asynchronous
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
    t_last = t_now;
    t_now = micros();
    Serial.print("Sample rate = ");
    Serial.println(1e6/(t_now - t_last));
  }

}

