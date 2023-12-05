#include "Quadrant.h"
#include <Wire.h>
#include <ArduinoJson.h>

SerialPIO softSerial(11, SerialPIO::NOPIN);
MIDI_NAMESPACE::SerialMIDI<SerialPIO> softSerialMidi(softSerial);
MIDI_CREATE_INSTANCE(SerialPIO, softSerialMidi, QMIDI)

void Quadrant::begin(){

  // initialize private variables
  for (int i=0; i<4; i++) {
    _distance[i] = 0xff;
    _engaged[i] = false;
    _lidarEnabled[i] = false;
  }

  _thresh = DEFAULT_ENGAGEMENT_THRESHOLD;
  _smode = QUADRANT_SAMPLINGMODE_CONTINUOUS_TIMED;
  _filter_enabled = false;

  // LEDS
  for (int i=0; i<4; i++) {
    pinMode(_ledPins[i], OUTPUT); 
    digitalWrite(_ledPins[i], LOW);
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

  // DAC
  Wire1.setSDA(6);
  Wire1.setSCL(7);
  Wire1.begin();

  // MIDI
  softSerial.setInverted(true,true);
  QMIDI.begin();


  // sample timer
  _tlast = micros();
  _tnow = micros();

}

void Quadrant::setLidarEnabled(int index, bool enabled) {

  _lidarEnabled[index] = enabled;

}

bool Quadrant::isLidarEnabled(int index) {

  return _lidarEnabled[index];

}

void Quadrant::setEngagementThreshold(int value) {

  _thresh = value;

}

void Quadrant::update(void) {

  switch (_smode) {
    case QUADRANT_SAMPLINGMODE_SINGLE_SEQUENTIAL:
      break;
    case QUADRANT_SAMPLINGMODE_SINGLE_PIPELINE:
      _update_single_pipeline();
      break;
    case QUADRANT_SAMPLINGMODE_CONTINUOUS:
      _update_continuous();
      break;
    case QUADRANT_SAMPLINGMODE_CONTINUOUS_TIMED:
      _update_continuous();
      break;
    default:
      break;
  }

}


void Quadrant::updateFilter(void) {

  for (int i=0; i<4; i++) {
    _filter[_ifilter*4 + i] = _distance[i];
  }

  _ifilter += 1;
  if (_ifilter >= _len_filter) _ifilter = 0;

}

void Quadrant::initFilter(uint8_t len) {

  // simple boxcar filter
  _filter = (uint16_t*) malloc(len * 4 * sizeof(uint16_t));

  for (int i=0; i<(len*4); i++) {
      _filter[i] = 0xffff;
  }

  _len_filter = len;
  _ifilter = 0;

  _filter_enabled = true;

}


// getters

float Quadrant::getSampleRate(void) {

  return 1e6 / float(_tnow - _tlast);

}

bool Quadrant::isLidarEngaged(int index) {

  return _engaged[index];

}

uint16_t Quadrant::getLidarDistance(int index) {

  // distance in mm

  return _distance[index];

}

float Quadrant::getLidarDistanceNormalized(int index) {

  // unit-less distance normalized to the engagement threshold

  return float(_distance[index]) / _thresh;

}

float Quadrant::getLidarDistanceFiltered(int index) {

  long tmp = 0;

  for (int j=0; j<_len_filter; j++) {
      tmp += _filter[j*4 + index];
  }

  return float(tmp) / _len_filter;

}

bool Quadrant::isElevationEngaged(void) {

  return _engaged[0] && _engaged[1] && _engaged[2] && _engaged[3];

}

float Quadrant::getElevation(void) {

  // returns a value between 0 and 1

  int count = 0;
  float total_distance = 0;

  for (int i=0; i<4; i++) {
    if (isLidarEngaged(i)) {
      if (_filter_enabled) {
        total_distance += getLidarDistanceFiltered(i);      
      } else {
        total_distance += getLidarDistance(i);      
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

bool Quadrant::isPitchEngaged(void) {

  return _engaged[1] && _engaged[3];

}

float Quadrant::getPitch(void) {

  // returns a value between -1 and 1

  if (_filter_enabled) {
    return (getLidarDistanceFiltered(1) - getLidarDistanceFiltered(3)) / _thresh;
  } else {
    return float(_distance[1] - _distance[3]) / _thresh;
  }

}

bool Quadrant::isRollEngaged(void) {

  return _engaged[0] && _engaged[2];

}

float Quadrant::getRoll(void) {

  // returns a value between -1 and 1

  if (_filter_enabled) {
    return (getLidarDistanceFiltered(0) - getLidarDistanceFiltered(2)) / _thresh;
  } else {
    return float(_distance[0] - _distance[2]) / _thresh;
  }

}

bool Quadrant::isArcEngaged(void) {

  return _engaged[0] && _engaged[1] && _engaged[2] && _engaged[3];

}

float Quadrant::getArc(void) {

  // returns a value between -1 and 1

  if (_filter_enabled) {
    return (getLidarDistanceFiltered(0) - getLidarDistanceFiltered(1)
            + getLidarDistanceFiltered(2) - getLidarDistanceFiltered(3)) / _thresh;
  } else {
    return float(_distance[0] - _distance[1] + _distance[2] - _distance[3]) / (2 * _thresh);
  }

}

double _round3(double value) {

    // convenience function for limiting sig figs in json serialization

   return (int)(value * 1000 + 0.5) / 1000.0;

}

void Quadrant::printReportToSerial(void) {

  StaticJsonDocument<512> report;

  JsonObject lidar0 = report.createNestedObject("lidar0");
  //lidar0["distance"] = getLidarDistance(0);
  lidar0["distance"] = _round3(getLidarDistanceFiltered(0));
  lidar0["engaged"] = isLidarEngaged(0);

  JsonObject lidar1 = report.createNestedObject("lidar1");
  //lidar1["distance"] = getLidarDistance(1);
  lidar1["distance"] = _round3(getLidarDistanceFiltered(1));
  lidar1["engaged"] = isLidarEngaged(1);

  JsonObject lidar2 = report.createNestedObject("lidar2");
  //lidar2["distance"] = getLidarDistance(2);
  lidar2["distance"] = _round3(getLidarDistanceFiltered(2));
  lidar2["engaged"] = isLidarEngaged(2);

  JsonObject lidar3 = report.createNestedObject("lidar3");
  //lidar3["distance"] = getLidarDistance(3);
  lidar3["distance"] = _round3(getLidarDistanceFiltered(3));
  lidar3["engaged"] = isLidarEngaged(3);

  JsonObject elevation = report.createNestedObject("elevation");
  elevation["value"] = _round3(getElevation());
  elevation["engaged"] = isElevationEngaged();

  JsonObject pitch = report.createNestedObject("pitch");
  pitch["value"] = _round3(getPitch());
  pitch["engaged"] = isPitchEngaged();

  JsonObject roll = report.createNestedObject("roll");
  roll["value"] = _round3(getRoll());
  roll["engaged"] = isRollEngaged();

  JsonObject arc = report.createNestedObject("arc");
  arc["value"] = _round3(getArc());
  arc["engaged"] = isArcEngaged();

  report["sampleRate"] = _round3(getSampleRate());

  serializeJson(report, Serial);
  Serial.println();

}

// outputs

void Quadrant::setLed(int index, int state) {

  digitalWrite(_ledPins[index], state);

}

void Quadrant::setCV(int chan, float voltage) {

  voltage = constrain(voltage, 0.0, 5.0);

  _writeDac(chan, voltage * 1023 / 5);

}

void Quadrant::sendMidiNoteOn(uint8_t note, uint8_t vel, uint8_t chan){

  QMIDI.sendNoteOn(note, vel, chan);

}

void Quadrant::sendMidiNoteOff(uint8_t note, uint8_t chan){

  QMIDI.sendNoteOff(note, 0, chan);

}

void Quadrant::sendMidiControlChange(uint8_t control_number, uint8_t val, uint8_t chan){

  QMIDI.sendControlChange(control_number, val, chan);

}

// private methods below

void Quadrant::_initLidar(int index) {

    digitalWrite(_ledPins[index], HIGH);

    digitalWrite(_lidarPins[index], HIGH);
    delay(100);

    _lidars[index] = new Adafruit_VL53L0X();

    if(!_lidars[index]->begin(_lidarAddrs[index])) {
      Serial.print(F("Failed to boot VL53L0X "));
      Serial.println(index);
    }

    _lidars[index]->configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);

    setLidarEnabled(index, true);

    if ((_smode == QUADRANT_SAMPLINGMODE_CONTINUOUS)
          || (_smode == QUADRANT_SAMPLINGMODE_CONTINUOUS_TIMED)) {

      _lidars[index]->startRangeContinuous(10);

    }

    digitalWrite(_ledPins[index], LOW);

}

bool Quadrant::_isLidarReady(uint8_t index){

  bool complete;

  complete = _lidars[index]->isRangeComplete();

  if (_lidars[index]->Status != VL53L0X_ERROR_NONE) {
    Serial.println("isRangeComplete failed");
    return false;
  }

  return complete;

}

uint16_t Quadrant::_readLidar(uint8_t index){

  uint16_t d = 0xff;

  d = _lidars[index]->readRangeResult();

  if (_lidars[index]->Status != VL53L0X_ERROR_NONE) {
    Serial.println("readRangeResult failed");
  }

  return d;

}

void Quadrant::_writeDac(uint8_t chan, int value){

  if (value > 1023) {
    value = 1023;
  }

  Wire1.beginTransmission(byte(0x58));
  Wire1.write(_dacAddrs[chan]);
  Wire1.write(value >> 2);
  Wire1.write((value & 0x03) << 6);
  Wire1.endTransmission();

}

void Quadrant::_update_single_pipeline(void) {

  bool done[4] = {false, false, false, false};

  // start all measuring at once
  for (int i=0; i<4; i++) {
    _lidars[i]->startMeasurement();
  }

  // poll in a loop, settings flags as readouts return
  while (!(done[0] && done[1] && done[2] && done[3])) {
    for (int i=0; i<4; i++) {
      if (!done[i]) {
        if (_lidars[i]->isRangeComplete()) {
          _distance[i] = _lidars[i]->readRangeResult();  // modified lib
          _engaged[i] = (_distance[i] < _thresh);
          done[i] = true;
        }
      }
    }
  }

  _tlast = _tnow;
  _tnow = micros();

}

void Quadrant::_update_continuous(void) {

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

  _tlast = _tnow;
  _tnow = micros();

}

