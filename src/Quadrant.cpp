#include <Wire.h>
#include "Quadrant.h"

void Quadrant::begin(void){

  // basic single-core setup()

  daq.begin();
  dsp.begin();
  out.begin();
  out.displayStartupLeds();

}

void Quadrant::update(void){

  // basic single-core loop():
  //  call this as quickly as you like (it will block until data is ready)
  //  then you can access:
  //    - quadrant.dsp GETTERS
  //    - quadrant.out OUTPUTS

  daq.update();
  dsp.update(&daq);
  out.updateReport(&dsp);

}

