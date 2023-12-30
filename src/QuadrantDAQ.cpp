#include <Wire.h>
#include "QuadrantDAQ.h"
#include "QuadrantOut.h"

#define QUADRANT_TIMING_BUDGET_US 20000
#define QUADRANT_SAMPLING_PERIOD_MS 24

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
    digitalWrite(_lidarGPIO[i], HIGH); // high Z
    pinMode(_lidarGPIO[i], INPUT);    // high Z
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

void QuadrantDAQ::setLidarEnabled(int index, bool enabled) {

  _lidarEnabled[index] = enabled;

}

void QuadrantDAQ::update(QuadrantOut *debug_out) {

  // updates _distance[i] and _timestamp, depending on the sampling mode
  // 
  //  optionally, set debug_out to the QuadrantOut* submodule to enable in-loop
  //  debugging via gate signals during while loops. currently sets channel gate
  //  to high while ranging. defaults to NULL (no debugging)

  switch (_smode) {
    case SAMPLINGMODE_SINGLE_SEQUENTIAL:
      _update_single_sequential();
      break;
    case SAMPLINGMODE_SINGLE_PIPELINE:
      _update_single_round_robin(debug_out);
      break;
    case SAMPLINGMODE_CONTINUOUS:
    case SAMPLINGMODE_PERIODIC:
      _update_continuous_round_robin(debug_out);
      break;
    default:
      break;
  }

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

int QuadrantDAQ::getSamplingMode(void) {

  return _smode;

}

uint16_t QuadrantDAQ::getLidarDistance(int index) {

  // distance in mm

  return _distance[index];

}

uint32_t QuadrantDAQ::getTimestamp(void) {

  return _timestamp;

}

bool QuadrantDAQ::isLidarEnabled(int index) {

  return _lidarEnabled[index];

}

bool QuadrantDAQ::isLidarRanging(uint8_t index) {

  return (!(_lidars[index]->readReg(VL53L0X::RESULT_RANGE_STATUS) & 0x01));

}

void QuadrantDAQ::waitUntilAllRanging(QuadrantOut *debug_out) {

  // blocks until all *enabled* lidars are in a ranging state
  // useful for syncing up timestamps for board samples

  bool done[4];

  for (int i=0; i<4; i++) {
    done[i] = !isLidarEnabled(i);
  }

  while (!(done[0] && done[1] && done[2] && done[3])) {
    for (int i=0; i<4; i++) {
      if (!done[i]) {
        done[i] = isLidarRanging(i);
        if (debug_out) debug_out->setGate(i, done[i]);
      }
    }
  }

}

uint8_t QuadrantDAQ::getTimeoutMask(void) {

  return _timeout_mask;

}

uint32_t QuadrantDAQ::getLidarTimingBudget(int index) {

  // the actual timing budget is often different from what's requested
  // this allows to to confirm the actual timing budget on each sensor

  return _lidars[index]->getMeasurementTimingBudget();

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
    _lidars[index]->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 12);
    _lidars[index]->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 8);
    _lidars[index]->setMeasurementTimingBudget(QUADRANT_TIMING_BUDGET_US);
    _lidars[index]->setAddress(_lidarAddrs[index]);

    setLidarEnabled(index, true);

    if (_smode == SAMPLINGMODE_CONTINUOUS) {
      _lidars[index]->startContinuous();
    } else if (_smode == SAMPLINGMODE_PERIODIC) {
      _lidars[index]->startContinuous(QUADRANT_SAMPLING_PERIOD_MS);
    }

}

bool QuadrantDAQ::_isLidarReady(uint8_t index){

  return (_lidars[index]->readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07);

}

uint16_t QuadrantDAQ::_readLidar(uint8_t index){

  uint16_t d;

  d = _lidars[index]->readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  _lidars[index]->writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);

  return d;

}

void QuadrantDAQ::_update_single_sequential(void) {

  uint16_t d;

  _timestamp = micros();

  _timeout_mask = 0;
  for (int i=0; i<4; i++) {
    if (isLidarEnabled(i)) {
      d = _lidars[i]->readRangeSingleMillimeters();
      if (_lidars[i]->timeoutOccurred()) {
        _timeout_mask |= (1 << i);
        continue;
      }
      _distance[i] = d;
    }
  }

}

void QuadrantDAQ::_update_continuous_sequential(QuadrantOut *debug_out) {

  uint16_t d;

  waitUntilAllRanging(debug_out);
  _timestamp = micros();

  _timeout_mask = 0;
  for (int i=0; i<4; i++) {
    if (isLidarEnabled(i)) {
      d = _lidars[i]->readRangeContinuousMillimeters();
      if (_lidars[i]->timeoutOccurred()) {
        _timeout_mask |= (1 << i);
        continue;
      }
      _distance[i] = d;
    }
  }

}

void QuadrantDAQ::_update_single_round_robin(QuadrantOut *debug_out) {

  // start all (enabled) sensors ranging
  for (int i=0; i<4; i++) {
    if (isLidarEnabled(i)) {
      _lidars[i]->writeReg(VL53L0X::SYSRANGE_START, 0x01);
    } else {
    }
  }

  // clock the timestamp
  _timestamp = micros();

  // collect _distance[i]
  _collect_range_round_robin(debug_out);


}

void QuadrantDAQ::_update_continuous_round_robin(QuadrantOut *debug_out) {

  waitUntilAllRanging(debug_out);
  _timestamp = micros();

  _collect_range_round_robin(debug_out);

}

void QuadrantDAQ::_collect_range_round_robin(QuadrantOut *debug_out) {

  // goes around all enabled sensors in a while loop, polling for readiness and
  // collecting their range measurements into _distance[i]
  //
  // obeys QUADRANT_TIMEOUT_MS and sets _timeout_mask
  //
  // follows the debug_out paradigm (default NULL, no debugging)

  bool done[4];
  unsigned long t0;

  for (int i=0; i<4; i++) {
    done[i] = !isLidarEnabled(i);
  }

  _timeout_mask = 0;
  t0 = millis();
  while (!(done[0] && done[1] && done[2] && done[3])) {
    for (int i=0; i<4; i++) {
      if (debug_out) debug_out->setGate(i, isLidarRanging(i));
      if (!done[i]) {
        if (_isLidarReady(i)) {
          _distance[i] = _readLidar(i);
          done[i] = true;
        } else if ((millis() - t0) > QUADRANT_TIMEOUT_MS) {
          _distance[i] = 0xffff;
          done[i] = true;
          _timeout_mask |= (1 << i);
          // TODO: cleanup? restart the measurement?
        }
      }
    }
  }

}

