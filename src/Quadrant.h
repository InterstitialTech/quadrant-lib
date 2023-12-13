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

    void begin(void);

    // Modules
    QuadrantDAQ daq;
    QuadrantDSP dsp;
    QuadrantOut out;

};


#endif
