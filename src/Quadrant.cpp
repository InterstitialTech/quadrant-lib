#include <Wire.h>
#include "Quadrant.h"

void Quadrant::begin(void) {

  daq.begin();
  dsp.begin();
  out.begin();
  out.displayStartupLeds();

}

void Quadrant::run(void) {

  // send the go cue to the other core

  rp2040.fifo.push(0xdeadbeef);
  rp2040.fifo.push((uint32_t)this);

}

bool Quadrant::newDataReady(void) {

	// NOTE: the default fifo size is only 8 (TODO: increase)

  return (rp2040.fifo.available() >= 4);

}

void Quadrant::update(void) {

	// grab next data packet
	// update DSP machine
	// update report
	// (TODO: add dataOrder = (next || last) argument)

  dsp.updateFromFifo();
  out.updateReport(&dsp);

}

uint16_t Quadrant::getLidarDistance(int index) {

	return dsp.getLidarDistance(index);

}


bool Quadrant::isLidarEngaged(int index) {

	return dsp.isLidarEngaged(index);

}

void Quadrant::setLed(int chan, int state) {

	return out.setLed(chan, state);

}

void Quadrant::setCV(int chan, float voltage) {

	out.setCV(chan, voltage);

}

void Quadrant::printReportToSerial(void) {

	out.printReportToSerial();

}

#ifndef QUADRANT_CUSTOM_MULTICORE

#define SAMPLE_PERIOD_US 23000

static Quadrant *_quadrant;
static unsigned long _tnow, _tlast=0;

void setup1(void) {

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

}

void loop1(void) {

  _tnow = micros();
  if ((_tnow - _tlast) > SAMPLE_PERIOD_US) {
    _quadrant->daq.update();
    _quadrant->daq.pushToFifo();
    _tlast = _tnow;
  }

}

#endif
