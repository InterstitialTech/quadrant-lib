/*
  Quadrant.h - Library for using Quadrant
  Created by Josh Muffin Gordonson (Interstitial Technology) on July 26 2023
  Released under the CERN OHL 2.0
*/

#ifndef Quadrant_h
#define Quadrant_h

#include "QuadrantDAQ.h"
#include "QuadrantDSP.h"
#include "QuadrantOut.h"

class Quadrant {

  public:

    // setup and loop
    void begin(void);
    void run(void);
		bool newDataReady(void);
    void update(void);

		// data getters
		bool getLidarDistance(int index);
		bool isLidarEngaged(int index);

		// output setters
		void setLed(int chan, int state);
		void setCV(int chan, float voltage);
		void printReportToSerial(void);

    // submodules
    QuadrantDAQ daq;
    QuadrantDSP dsp;
    QuadrantOut out;

};


// TODO: move this stuff to a new "multicore.h" header, switch on ifdef

#define SAMPLE_PERIOD_US 23000

void setup1(void) {

	Quadrant *_quadrant;
	static unsigned long tnow, tlast=0;
  uint32_t word;

  // block until we receive the go cue from the other core
  while (1) {
    word = rp2040.fifo.pop();
    if (word == 0xdeadbeef) {
      word = rp2040.fifo.pop();
      _quadrant = (Quadrant*) word;
      break;
    }
  }

	// core 1 loop
  while (1) {
		tnow = micros();
		if ((tnow - tlast) > SAMPLE_PERIOD_US) {
			_quadrant->daq.update();
			_quadrant->daq.pushToFifo();
			tlast = tnow;
		}
	}

}

#endif
