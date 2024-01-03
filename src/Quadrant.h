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
		uint16_t getLidarDistance(int index);
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


#endif
