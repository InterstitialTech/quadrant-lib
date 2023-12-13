#include <Wire.h>
#include "QuadrantDAQ.h"


void QuadrantDAQ::begin(void){

  Wire.setClock(400000);
  Wire.begin();

  // initialize private variables
  for (int i=0; i<4; i++) {
    _distance[i] = 0xff;
    _engaged[i] = false;
    _lidarEnabled[i] = false;
  }

  _thresh = QUADRANT_THRESH_DEFAULT_MM;
  _smode = SAMPLINGMODE_CONTINUOUS;

  // Lidars
  for (int i=0; i<4; i++) {
    pinMode(_lidarPins[i], OUTPUT);
    digitalWrite(_lidarPins[i], LOW);    
  }
  delay(100);
  for (int i=0; i<4; i++) {
    _initLidar(i);
  }

}

void QuadrantDAQ::setLidarEnabled(int index, bool enabled) {

  _lidarEnabled[index] = enabled;

}

bool QuadrantDAQ::isLidarEnabled(int index) {

  return _lidarEnabled[index];

}

void QuadrantDAQ::setEngagementThreshold(uint16_t thresh_mm) {

  _thresh = thresh_mm;

}

void QuadrantDAQ::update(void) {

  switch (_smode) {
    case SAMPLINGMODE_SINGLE_SEQUENTIAL:
      _update_single_sequential();
      break;
    case SAMPLINGMODE_SINGLE_PIPELINE:
      break;
    case SAMPLINGMODE_CONTINUOUS:
      _update_continuous_round_robin();
      break;
    case SAMPLINGMODE_CONTINUOUS_TIMED:
      break;
    default:
      break;
  }

}

void QuadrantDAQ::pushFrame(void) {

    for (int i=0; i<4; i++) {
      rp2040.fifo.push_nb(getLidarDistance(i));
    }

}

// getters

bool QuadrantDAQ::isLidarEngaged(int index) {

  return _engaged[index];

}

uint16_t QuadrantDAQ::getLidarDistance(int index) {

  // distance in mm

  return _distance[index];

}

// private methods below

void QuadrantDAQ::_initLidar(int index) {

    digitalWrite(_ledPins[index], HIGH);

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

    if ((_smode == SAMPLINGMODE_CONTINUOUS)
          || (_smode == SAMPLINGMODE_CONTINUOUS_TIMED)) {

      _lidars[index]->startContinuous();

    }
    digitalWrite(_ledPins[index], LOW);

}

bool QuadrantDAQ::_isLidarReady(uint8_t index){

  return (_lidars[index]->readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07);

}

uint16_t QuadrantDAQ::_readLidar(uint8_t index){

  uint16_t d = 0xff;

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
      _engaged[i] = (_distance[i] < _thresh);
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
      _engaged[i] = (_distance[i] < _thresh);
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
          _engaged[i] = (_distance[i] < _thresh);
          done[i] = true;
        }
      }
    }
  }

}

