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

*/

#define CV_GAIN_ELEVATION 1.0
#define CV_GAIN_PITCH  3.0
#define CV_GAIN_ROLL   4.0
#define CV_GAIN_ARC    10.0
#define MIDI_CHAN 1

#include "Quadrant.h"
Quadrant quadrant;

int midi_notes[4] = {60, 64, 67, 70};
bool was_engaged[4] = {false};
unsigned long tnow, tlast;

void setup(void) {

  quadrant.begin();
  quadrant.run();

}

void loop() {

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

    // output MIDI notes
    for (int i=0; i<4; i++) {
      if (quadrant.isLidarEngaged(i)) {
        if (!was_engaged[i]) {
          Serial.println("sending note on");
          quadrant.sendMidiNoteOn(midi_notes[i], 127, MIDI_CHAN);
        }
        was_engaged[i] = true;
      } else {
        if (was_engaged[i]) {
          Serial.println("sending note off");
          quadrant.sendMidiNoteOff(midi_notes[i], MIDI_CHAN);
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

