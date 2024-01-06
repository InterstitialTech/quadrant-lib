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

  Beam crossings generate MIDI notes on the TRS MIDI jack. The set of MIDI
  notes can be selected "in the field" via the 2-channel DIP switch (4 options).

  To change the note values or MIDI channel, set midi_notes[] and MIDI_CHAN
  respectively.

*/

#define CV_GAIN_ELEVATION 1.0
#define CV_GAIN_PITCH  3.0
#define CV_GAIN_ROLL   4.0
#define CV_GAIN_ARC    10.0
#define MIDI_CHAN 1

#include "Quadrant.h"
Quadrant quadrant;

int midi_notes_0[4] = {48, 52, 55, 59};   // 1MD7
int midi_notes_1[4] = {60, 64, 67, 71};   // 2mD7
int midi_notes_2[4] = {72, 76, 79, 83};   // 4MD7
int midi_notes_3[4] = {84, 88, 91, 95};   // 5MD7

void setup(void) {

  quadrant.begin();
  quadrant.enableFilter(true);
  quadrant.run();

}

void loop() {

  static bool was_engaged[4] = {false};

  int *midi_notes;
  uint8_t dip;

  if (quadrant.newDataReady()) {

		// update local state machine and status report
		quadrant.update();

    // set LEDs and Gates according to engagement
		for (int i=0; i<4; i++) {
			if (quadrant.isLidarEngaged(i)) {
				quadrant.setLed(i, HIGH);
        quadrant.setGate(i, HIGH);
			} else {
				quadrant.setLed(i, LOW);
        quadrant.setGate(i, LOW);
			}
		}

    //  output elevation to CV 0
    if (quadrant.isElevationEngaged()) {
      quadrant.setCV(0, quadrant.getElevation() * 5.0 * CV_GAIN_ELEVATION);
    }
    //  output pitch to CV 1
    if (quadrant.isPitchEngaged()) {
      quadrant.setCV(1, 2.5 + quadrant.getPitch() * 2.5 * CV_GAIN_PITCH);
    }
    //  output roll to CV 2
    if (quadrant.isRollEngaged()) {
      quadrant.setCV(2, 2.5 + quadrant.getRoll() * 2.5 * CV_GAIN_ROLL);
    }
    //  output arc to CV 3
    if (quadrant.isArcEngaged()) {
      quadrant.setCV(3, 2.5 + quadrant.getArc() * 2.5 * CV_GAIN_ARC);
    }

    // select midi notes based on DIP switches
    dip = (quadrant.dip1() << 1) | (quadrant.dip2() << 0);
    switch (dip) {
      case 0:
        midi_notes = midi_notes_0;
        break;
      case 1:
        midi_notes = midi_notes_1;
        break;
      case 2:
        midi_notes = midi_notes_2;
        break;
      case 3:
        midi_notes = midi_notes_3;
        break;
    }

    // detect note on, note off. send over MIDI and USB report
    char event[16];
    for (int i=0; i<4; i++) {
      if (quadrant.isLidarEngaged(i)) {
        if (!was_engaged[i]) {
          quadrant.sendMidiNoteOn(midi_notes[i], 127, MIDI_CHAN);
          sprintf(event, "on_%d", i);
          quadrant.out.reportEvent(event);
        }
        was_engaged[i] = true;
      } else {
        if (was_engaged[i]) {
          quadrant.sendMidiNoteOff(midi_notes[i], MIDI_CHAN);
          sprintf(event, "off_%d", i);
          quadrant.out.reportEvent(event);
        }
        was_engaged[i] = false;
      }
    }

		// print a JSON status report to the USB serial monitor
		quadrant.printReportToSerial();

  }

  // pass MIDI thru (from in -> out)
  quadrant.handleMidiThru();

}

