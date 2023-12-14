#include <Wire.h>
#include "QuadrantDAQ.h"


void QuadrantDAQ::begin(void){

  Wire.setClock(400000);
  Wire.begin();

  // initialize private variables
  for (int i=0; i<4; i++) {
    _distance[i] = 0xffff;
    _lidarEnabled[i] = false;
  }

  // Lidars
  for (int i=0; i<4; i++) {
    pinMode(_lidarPins[i], OUTPUT);
    digitalWrite(_lidarPins[i], LOW);    
  }
  delay(100);
  for (int i=0; i<4; i++) {
    _initLidar(i);
  }

  _timestamp = 0;

}

void QuadrantDAQ::setSamplingMode(enum SamplingMode mode) {

  // NOTE: after setting the sampling mode, you must then call QuadrantDAQ::begin()
  // in order for it to take effect!

  _smode = mode;

}

int QuadrantDAQ::getSamplingMode(void) {

  return _smode;

}

void QuadrantDAQ::setLidarEnabled(int index, bool enabled) {

  _lidarEnabled[index] = enabled;

}

bool QuadrantDAQ::isLidarEnabled(int index) {

  return _lidarEnabled[index];

}

void QuadrantDAQ::update(void) {

  switch (_smode) {
    case SAMPLINGMODE_SINGLE_SEQUENTIAL:
      _update_single_sequential();
      break;
    case SAMPLINGMODE_SINGLE_PIPELINE:
      // TODO
      break;
    case SAMPLINGMODE_CONTINUOUS:
      _update_continuous_round_robin();
      break;
    case SAMPLINGMODE_PERIODIC:
      _update_continuous_round_robin();
      break;
    default:
      break;
  }

  _timestamp = micros();

}

void QuadrantDAQ::pushToFifo(void) {

    uint32_t e;

    // first byte is 0xdeadbeef, signaling packet start
    rp2040.fifo.push_nb(0xdeadbeef);

    // second byte starts with 100, signaling channels 0,1
    e = (0b100) << 29;
    e |= (getLidarDistance(0) & 0x1fff) << 16;
    e |= (getLidarDistance(1) & 0x1fff) << 0;
    rp2040.fifo.push_nb(e);

    // third byte starts with 101, signaling channels 2,3
    e = (0b101) << 29;
    e |= (getLidarDistance(2) & 0x1fff) << 16;
    e |= (getLidarDistance(3) & 0x1fff) << 0;
    rp2040.fifo.push_nb(e);

    // fourth byte is the entire timestamp (32 bits)
    rp2040.fifo.push_nb(_timestamp);

}

// getters

uint16_t QuadrantDAQ::getLidarDistance(int index) {

  // distance in mm

  return _distance[index];

}

uint32_t QuadrantDAQ::getTimestamp(void) {

  return _timestamp;

}

// private methods below

void QuadrantDAQ::_initLidar(int index) {

    digitalWrite(_lidarPins[index], HIGH);
    delay(100);

    _lidars[index] = new VL53L0X();
    _lidars[index]->setTimeout(QUADRANT_TIMEOUT_MS);
    if(!_lidars[index]->init()) {
      Serial.print(F("Failed to boot VL53L0X #"));
      Serial.println(index);
    }
    _lidars[index]->setMeasurementTimingBudget(24000);
    _lidars[index]->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 12);
    _lidars[index]->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 8);
    _lidars[index]->setAddress(_lidarAddrs[index]);

    setLidarEnabled(index, true);

    if (_smode == SAMPLINGMODE_CONTINUOUS) {
      _lidars[index]->startContinuous();
    } else if (_smode == SAMPLINGMODE_PERIODIC) {
      _lidars[index]->startContinuous(24);
    }

}

bool QuadrantDAQ::_isLidarReady(uint8_t index){

  return (_lidars[index]->readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07);

}

uint16_t QuadrantDAQ::_readLidar(uint8_t index){

  uint16_t d;

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled

  d = _lidars[index]->readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  _lidars[index]->writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);

  return d;

}

void QuadrantDAQ::_update_single_sequential(void) {

  uint16_t d;

  for (int i=0; i<4; i++) {
    if (isLidarEnabled(i)) {
      d = _lidars[i]->readRangeSingleMillimeters();
      if (_lidars[i]->timeoutOccurred()) {
        Serial.println("timeout occurred!");
        continue;
      }
      _distance[i] = d;
    }
  }

}

void QuadrantDAQ::_update_continuous_sequential(void) {

  uint16_t d;

  for (int i=0; i<4; i++) {
    if (isLidarEnabled(i)) {
      d = _lidars[i]->readRangeContinuousMillimeters();
      if (_lidars[i]->timeoutOccurred()) {
        Serial.println("timeout occurred!");
        continue;
      }
      _distance[i] = d;
    }
  }

}

void QuadrantDAQ::_update_continuous_round_robin(void) {

  // note: no timeout checking here

  bool done[4];

  for (int i=0; i<4; i++) {
    done[i] = !isLidarEnabled(i);
  }

  while (!(done[0] && done[1] && done[2] && done[3])) {
    for (int i=0; i<4; i++) {
      if (!done[i]) {
        if (_isLidarReady(i)) {
          _distance[i] = _readLidar(i);
          done[i] = true;
        }
      }
    }
  }

}

