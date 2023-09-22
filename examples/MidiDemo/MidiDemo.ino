/*
  Send a MIDI on+off when a channel's beam is crossed.
*/

#include "quadrant.h"

#define LED_THRESH 180

Quadrant myquad;

int values[4] = {8192, 8192, 8192, 8192};
char serial_buf[21];
int notes[4] = {60, 65, 67, 70};

void setup() {

  Serial.begin(115200);
  //while(!Serial);

  myquad.begin();
  myquad.setLidarsContinuous();

}

void loop() {

  int value;
  bool newValue;
  bool crossed;

  newValue = false;
  bool engaged[4] = {false};
  bool triggered[4] = {false};

  for(int i=0; i<4; i++){
    if (myquad.checkLidarContinuous(i)) {
      value = myquad.readLidarContinuous(i);
      triggered[i] = false;
      if (value < LED_THRESH) {
        if (!engaged[i]) {
          triggered[i] = true;
        }
        engaged[i] = true;
        digitalWrite(myquad.leds[i], HIGH);
      } else {
        engaged[i] = false;
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
    for (int i=0; i<4; i++) {
      if (triggered[i]) {
        Serial.println("sending midi");
        myquad.sendMidiNoteOnOff(notes[i], 127, 1);
      }
    }
  }

}

