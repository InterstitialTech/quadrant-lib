/*

  MidiDemo.ino
  Chris Chronopoulos, 2023

  This sketch sends a MIDI-On followed by MIDI-Off when a channel's beam is
  crossed.

*/

#define MIDI_CHAN 1
int notes[4] = {60, 65, 67, 70};

#include "Quadrant.h"

Quadrant quadrant;

bool wasEngaged[4] = {false};


void setup() {

  quadrant.begin();
  quadrant.calibrateOffsets();
  Serial.begin(115200);

}


void loop() {

  uint8_t velocity;

  // take lidar measurement and update state variables
  quadrant.update();

  // set indicator leds
  for (int i=0; i<4; i++) {
    if (quadrant.isLidarEngaged(i)) {
      quadrant.setLed(i, HIGH);
    } else {
      quadrant.setLed(i, LOW);
    }
  }

  // trigger logic
  for(int i=0; i<4; i++){
    if (quadrant.isLidarEngaged(i)) {
      if (!wasEngaged[i]) {
        velocity = (uint8_t) 127 * (1 - quadrant.getLidarDistanceNormalized(i));
        quadrant.sendMidiNoteOn(notes[i], velocity, MIDI_CHAN);
      }
    } else {
      if (wasEngaged[i]) {
        quadrant.sendMidiNoteOff(notes[i], MIDI_CHAN);
      }
    }
    wasEngaged[i] = quadrant.isLidarEngaged(i);
  }
  
}

