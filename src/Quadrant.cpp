#include <Wire.h>
#include "Quadrant.h"

void Quadrant::begin(void){

  daq.begin();
  dsp.begin();
  out.begin();
  out.displayStartupLeds();

}

void Quadrant::update(void){

  daq.update();
  dsp.update(&daq);

}

