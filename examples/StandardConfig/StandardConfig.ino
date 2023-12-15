/*

  StandardConfig.ino
  Chris Chronopoulos, 2023

  This sketch represents the standard, out-of-the-box configuration for
  Quadrant. It measures all 4 lidar channels, and then computes the following
  aggregate parameters:

    * elevation: average height
    * pitch: front-to-back tilt
    * roll: side-to-side tilt
    * arc: curvature/cupping

  These parameters are then mapped to CV channels 0,1,2,3. The sensitivity of
  the CV outputs to each parameter can be adjusted by changing the #define
  params CV_GAIN_xx below.

  The lidar values, the sample rate, and aggregate parameters are all
  printed to USB serial in JSON format.

  Beam crossings generate MIDI notes on the TRS MIDI jack. To change the note
  values or MIDI channel, set midi_notes[] and MIDI_CHAN respectively.

  The board sample rate is 43.5 Hz (about as fast as possible for this sketch);
  you may adjust this using the SAMPLE_PERIOD_US parameter below.

  This is a multicore sketch - hence the loop() vs loop1(), etc.

*/

#define CV_GAIN_ELEVATION 1.0
#define CV_GAIN_PITCH  3.0
#define CV_GAIN_ROLL   4.0
#define CV_GAIN_ARC    10.0
#define SAMPLE_PERIOD_US 23000
#define MIDI_CHAN 1

#include "Quadrant.h"
Quadrant quadrant;

int midi_notes[4] = {60, 64, 67, 70};
bool was_engaged[4] = {false};
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
  quadrant.dsp.initFilter(4);

  quadrant.out.begin();
  quadrant.out.displayStartupLeds();
  quadrant.out.sendMidiControlChange(23, 0, 1);  // key sync off

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

  // output MIDI
  for (int i=0; i<4; i++) {
    if (quadrant.dsp.isLidarEngaged(i)) {
      if (!was_engaged[i]) {
        quadrant.out.sendMidiNoteOn(midi_notes[i], 127, MIDI_CHAN);
      }
      was_engaged[i] = true;
    } else {
      if (was_engaged[i]) {
        quadrant.out.sendMidiNoteOff(midi_notes[i], MIDI_CHAN);
      }
      was_engaged[i] = false;
    }
  }

  // output control voltages:

  //  elevation to CV 0
  if (quadrant.dsp.isElevationEngaged()) {
    quadrant.out.setCV(0, quadrant.dsp.getElevation() * 5.0 * CV_GAIN_ELEVATION);
  }
  //  pitch to CV 1
  if (quadrant.dsp.isPitchEngaged()) {
    quadrant.out.setCV(1, 2.5 + quadrant.dsp.getPitch() * 2.5 * CV_GAIN_PITCH);
  }
  //  roll to CV 2
  if (quadrant.dsp.isRollEngaged()) {
    quadrant.out.setCV(2, 2.5 + quadrant.dsp.getRoll() * 2.5 * CV_GAIN_ROLL);
  }
  //  arc to CV 3
  if (quadrant.dsp.isArcEngaged()) {
    quadrant.out.setCV(3, 2.5 + quadrant.dsp.getArc() * 2.5 * CV_GAIN_ARC);
  }

  // print status report to USB serial
  quadrant.out.updateReport(&quadrant.dsp);
  quadrant.out.printReportToSerial();

}

