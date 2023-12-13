#include "Arduino.h"
#include "QuadrantDSP.h"
#include "QuadrantCommon.h"


void QuadrantDSP::begin(void){

  // initialize private variables
  for (int i=0; i<4; i++) {
    _offset[i] = 0;
    _engaged[i] = false;
  }

  _thresh = QUADRANT_THRESH_DEFAULT_MM;
  _filter_enabled = false;

}

void QuadrantDSP::update(uint16_t *frame){

  for (int i=0; i<4; i++) {
    _distance[i] = frame[i];
    _engaged[i] = (_distance[i] < _thresh);
  }

  if (_filter_enabled) {
    updateFilter();
  }

}

void QuadrantDSP::popFrameMulticore(uint16_t *frame) {

  for (int i=0; i<4; i++) {
    frame[i] = (uint16_t) rp2040.fifo.pop();
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

void QuadrantDSP::initFilter(uint8_t len) {

  // simple boxcar filter
  _filter = (uint16_t*) malloc(len * 4 * sizeof(uint16_t));

  for (int i=0; i<(len*4); i++) {
      _filter[i] = 0xffff;
  }

  _len_filter = len;
  _ifilter = 0;

  _filter_enabled = true;

}

void QuadrantDSP::updateFilter(void) {

  for (int i=0; i<4; i++) {
    _filter[_ifilter*4 + i] = _distance[i];
  }

  _ifilter += 1;
  if (_ifilter >= _len_filter) _ifilter = 0;

}


float QuadrantDSP::getLidarDistanceFiltered(int index) {

  long tmp = 0;

  for (int j=0; j<_len_filter; j++) {
      tmp += _filter[j*4 + index];
  }

  return float(tmp) / _len_filter;

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
      if (_filter_enabled) {
        total_distance += getLidarDistanceFiltered(i);      
      } else {
        total_distance += _distance[i];      
      }
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

  if (_filter_enabled) {
    return atan2(getLidarDistanceFiltered(1) - getLidarDistanceFiltered(3),
                  QUADRANT_HEIGHT_MM) / (M_PI/2);
  } else {
    return atan2(_distance[1] - _distance[3], QUADRANT_HEIGHT_MM) / (M_PI/2);
  }

}

bool QuadrantDSP::isRollEngaged(void) {

  return _engaged[0] && _engaged[2];

}

float QuadrantDSP::getRoll(void) {

  // returns a value between -1 and 1

  if (_filter_enabled) {
    return atan2(getLidarDistanceFiltered(0) - getLidarDistanceFiltered(2),
                  QUADRANT_WIDTH_MM) / (M_PI/2);
  } else {
    return atan2(_distance[0] - _distance[2], QUADRANT_WIDTH_MM) / (M_PI/2);
  }

}

bool QuadrantDSP::isArcEngaged(void) {

  return _engaged[0] && _engaged[1] && _engaged[2] && _engaged[3];

}

float QuadrantDSP::getArc(void) {

  // returns a value between -1 and 1

  if (_filter_enabled) {
    return atan2(getLidarDistanceFiltered(0) - getLidarDistanceFiltered(1)
                  + getLidarDistanceFiltered(2) - getLidarDistanceFiltered(3),
                  QUADRANT_HEIGHT_MM) / (M_PI/2);
  } else {
    return atan2(_distance[0] - _distance[1] + _distance[2] - _distance[3],
                  QUADRANT_HEIGHT_MM) / (M_PI/2);
  }

}

