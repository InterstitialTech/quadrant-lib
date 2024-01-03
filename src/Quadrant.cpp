#include <Wire.h>
#include "Quadrant.h"

// setup, config, run

void Quadrant::begin(void) {

  daq.begin();
  dsp.begin();
  out.begin();
  out.displayStartupLeds();

}

void Quadrant::setEngagementThreshold(uint16_t value_mm) {

  dsp.setEngagementThreshold(value_mm);

}

void Quadrant::enableFilter(bool enabled) {

  dsp.enableFilter(enabled);

}

void Quadrant::configureReport(enum QUADRANT_REPORT_FIELD field, bool enabled) {

  out.configureReport(field, enabled);

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

// data getters

uint16_t Quadrant::getLidarDistance(int index) {

  return dsp.getLidarDistance(index);

}

bool Quadrant::isLidarEngaged(int index) {

	return dsp.isLidarEngaged(index);

}

bool Quadrant::isFilterEnabled(void) {

	return dsp.isFilterEnabled();

}

uint32_t Quadrant::getTimestamp(void) {

  return dsp.getTimestamp();

}

uint16_t Quadrant::getEngagementThreshold(void) {

  return dsp.getEngagementThreshold();

}

bool Quadrant::isElevationEngaged(void) {

  return dsp.isElevationEngaged();

}

float Quadrant::getElevation(void) {

  return dsp.getElevation();

}

bool Quadrant::isPitchEngaged(void) {

  return dsp.isPitchEngaged();

}

float Quadrant::getPitch(void) {

  return dsp.getPitch();

}

bool Quadrant::isRollEngaged(void) {

  return dsp.isRollEngaged();

}

float Quadrant::getRoll(void) {

  return dsp.getRoll();

}

bool Quadrant::isArcEngaged(void) {

  return dsp.isArcEngaged();

}

float Quadrant::getArc(void) {

  return dsp.getArc();

}

// output

void Quadrant::setLed(int chan, int state) {

	return out.setLed(chan, state);

}

void Quadrant::setCV(int chan, float voltage) {

	out.setCV(chan, voltage);

}

void Quadrant::setGate(int chan, int state) {

	out.setGate(chan, state);

}

void Quadrant::sendMidiNoteOn(uint8_t note, uint8_t vel, uint8_t chan) {

  out.sendMidiNoteOn(note, vel, chan);

}

void Quadrant::sendMidiNoteOff(uint8_t note, uint8_t chan) {

  out.sendMidiNoteOff(note, chan);

}

void Quadrant::sendMidiControlChange(uint8_t control_number, uint8_t val, uint8_t chan) {

  out.sendMidiControlChange(control_number, val, chan);

}

void Quadrant::handleMidiThru(void) {

  out.handleMidiThru();

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
