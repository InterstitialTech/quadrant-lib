#include <Wire.h>
#include "Quadrant.h"

void Quadrant::begin(void){

  daq.begin();
  dsp.begin();
  out.begin();

}

