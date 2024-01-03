#include "QuadrantDSP.h"


void QuadrantDSP::begin(void){

  // initialize private variables
  for (int i=0; i<4; i++) {
    _offset[i] = 0;
    _engaged[i] = false;
  }

  _thresh = QUADRANT_THRESH_DEFAULT_MM;
  _initFilter(QUADRANT_FILTER_LENGTH_DEFAULT);
  _filter_enabled = false;
  _timestamp = 0;

}

void QuadrantDSP::update(QuadrantDAQ *daq) {

  for (int i=0; i<4; i++) {
    _distance[i] = daq->getLidarDistance(i);
    _engaged[i] = (_distance[i] < _thresh);
  }

  _timestamp = daq->getTimestamp();

  if (_filter_enabled) {
    updateFilter();
  }

}

void QuadrantDSP::updateFromFifo(void) {

    uint32_t e;

    // first element
    e = rp2040.fifo.pop();
    // if (e != 0xdeadbeef) {}   // TODO: test for packet start

    // second element
    e = rp2040.fifo.pop();
    // if (!(e & (0b100 << 29))) {}   // TODO: test for channels 0,1
    _distance[0] = ((e >> 16) & 0x1fff);
    _distance[1] = (e & 0x1fff);

    // third element
    e = rp2040.fifo.pop();
    // if (!(e & (0b101 << 29))) {}   // TODO: test for channels 2,3
    _distance[2] = ((e >> 16) & 0x1fff);
    _distance[3] = (e & 0x1fff);

    // fourth element
    _timestamp = rp2040.fifo.pop();

  for (int i=0; i<4; i++) {
    _engaged[i] = (_distance[i] < _thresh);
  }

  if (_filter_enabled) {
    updateFilter();
  }

}

void QuadrantDSP::calibrateOffsets(void) {

  uint16_t min = 0xffff;

  if (!isElevationEngaged()) {
    return;
  }

  for (int i=0; i<4; i++) {
    if (_distance[i] < min) {
      min = _distance[i];
    }
  }

  for (int i=0; i<4; i++) {
    _offset[i] = _distance[i] - min;
  }

}

void QuadrantDSP::enableFilter(bool enabled) {

  _filter_enabled = enabled;

}

void QuadrantDSP::updateFilter(void) {

  for (int i=0; i<4; i++) {
    _filter[_ifilter*4 + i] = _distance[i];
  }

  _ifilter += 1;
  if (_ifilter >= _len_filter) _ifilter = 0;

}

void QuadrantDSP::setEngagementThreshold(uint16_t value_mm) {

  _thresh = value_mm;

}

// GETTERS

bool QuadrantDSP::isFilterEnabled(void) {

  return _filter_enabled;

}

uint32_t QuadrantDSP::getTimestamp(void) {

  return _timestamp;

}

float QuadrantDSP::getLidarDistance(int index) {

  // return distance in mm

  long tmp ;

  if (_filter_enabled) {

    tmp = 0;
    for (int j=0; j<_len_filter; j++) {
        tmp += _filter[j*4 + index];
    }

    return float(tmp) / _len_filter;

  } else {

    return _distance[index];

  }

}

float QuadrantDSP::getLidarDistanceNormalized(int index) {

  return float(getLidarDistance(index)) / _thresh;

}

uint16_t QuadrantDSP::getEngagementThreshold(void) {

  return _thresh;

}

bool QuadrantDSP::isLidarEngaged(int index) {

  return _engaged[index];

}

bool QuadrantDSP::isElevationEngaged(void) {

  return _engaged[0] && _engaged[1] && _engaged[2] && _engaged[3];

}

float QuadrantDSP::getElevation(void) {

  // returns a value between 0 and 1

  int count = 0;
  float total_distance = 0;

  for (int i=0; i<4; i++) {
    if (_engaged[i]) {
      total_distance += getLidarDistance(i);      
      count += 1;
    }
  }

  if (count == 0) {
    return 1.;
  } else {
    return total_distance / (count * _thresh);
  }

}

bool QuadrantDSP::isPitchEngaged(void) {

  return _engaged[1] && _engaged[3];

}

float QuadrantDSP::getPitch(void) {

  // returns a value between -1 and 1

  return atan2(getLidarDistance(1) - getLidarDistance(3),
                QUADRANT_HEIGHT_MM) / (M_PI/2);

}

bool QuadrantDSP::isRollEngaged(void) {

  return _engaged[0] && _engaged[2];

}

float QuadrantDSP::getRoll(void) {

  // returns a value between -1 and 1

  return atan2(getLidarDistance(0) - getLidarDistance(2),
                QUADRANT_WIDTH_MM) / (M_PI/2);

}

bool QuadrantDSP::isArcEngaged(void) {

  return _engaged[0] && _engaged[1] && _engaged[2] && _engaged[3];

}

float QuadrantDSP::getArc(void) {

  // returns a value between -1 and 1

  return atan2(getLidarDistance(0) - getLidarDistance(1)
                + getLidarDistance(2) - getLidarDistance(3),
                QUADRANT_HEIGHT_MM) / (M_PI/2);

}

// private

void QuadrantDSP::_initFilter(uint8_t len) {

  // simple boxcar filter
  _filter = (uint16_t*) malloc(len * 4 * sizeof(uint16_t));

  for (int i=0; i<(len*4); i++) {
      _filter[i] = 0xffff;
  }

  _len_filter = len;
  _ifilter = 0;

}

