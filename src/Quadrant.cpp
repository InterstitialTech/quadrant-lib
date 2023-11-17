#include "Quadrant.h"
#include <Wire.h>
#include <ArduinoJson.h>

SerialPIO softSerial(11, SerialPIO::NOPIN);
MIDI_NAMESPACE::SerialMIDI<SerialPIO> softSerialMidi(softSerial);
MIDI_CREATE_INSTANCE(SerialPIO, softSerialMidi, QMIDI)

Quadrant::Quadrant() {}

void Quadrant::begin(){

  // LEDS
  for (int i=0; i<4; i++) {
    pinMode(_ledPins[i], OUTPUT); 
  }

  // Lidars
  for (int i=0; i<4; i++) {
    pinMode(_lidarPins[i], OUTPUT);
    digitalWrite(_lidarPins[i], LOW);    
    _lidars[i] = new Adafruit_VL53L0X();
    _setLidarAddress(i);
    _setLidarContinuous(i);
  }

  // DAC
  Wire1.setSDA(6);
  Wire1.setSCL(7);
  Wire1.begin();

  // MIDI
  softSerial.setInverted(true,true);
  QMIDI.begin();

  // state variables
  for (int i=0; i<4; i++) {
    _distance[i] = 8192;
    _engaged[i] = false;
  }
  _thresh = DEFAULT_ENGAGEMENT_THRESHOLD;

  // sample timer
  _tlast = micros();
  _tnow = micros();

}

void Quadrant::setEngagementThreshold(int value) {

  _thresh = value;

}

void Quadrant::update(void) {

  bool done[4] = {false};

  while (!(done[0] && done[1] && done[2] && done[3])) {
    for (int i=0; i<4; i++) {
      if (!done[i]) {
        if (_isLidarReady(i)) {
          _distance[i] = _readLidar(i);
          _engaged[i] = (_distance[i] < 300);
          done[i] = true;
        }
      }
    }
  }

  _tlast = _tnow;
  _tnow = micros();

}

// getters

float Quadrant::getSampleRate(void) {

  return 1e6 / float(_tnow - _tlast);

}

bool Quadrant::isLidarEngaged(int index) {

  return _engaged[index];

}

int Quadrant::getLidarDistance(int index) {

  // distance in mm

  return _distance[index];

}

float Quadrant::getLidarDistanceNormalized(int index) {

  // unit-less distance normalized to the engagement threshold

  return float(_distance[index]) / _thresh;

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
      total_distance += getLidarDistanceNormalized(i);      
      count += 1;
    }
  }

  if (count == 0) {
    return 1.;
  } else {
    return total_distance / count;
  }

}

bool Quadrant::isPitchEngaged(void) {

  return _engaged[1] && _engaged[3];

}

float Quadrant::getPitch(void) {

  // returns a value between -1 and 1

    return float(_distance[1] - _distance[3]) / _thresh;

}

bool Quadrant::isRollEngaged(void) {

  return _engaged[0] && _engaged[2];

}

float Quadrant::getRoll(void) {

  // returns a value between -1 and 1

    return float(_distance[0] - _distance[2]) / _thresh;

}

bool Quadrant::isArcEngaged(void) {

  return _engaged[0] && _engaged[1] && _engaged[2] && _engaged[3];

}

float Quadrant::getArc(void) {

  // returns a value between -1 and 1

    return float(_distance[0] - _distance[1] + _distance[2] - _distance[3]) / (2 * _thresh);

}

double _round3(double value) {

    // convenience function for limiting sig figs in json serialization

   return (int)(value * 1000 + 0.5) / 1000.0;

}

void Quadrant::printReportToSerial(void) {

  StaticJsonDocument<512> report;

  JsonObject lidar0 = report.createNestedObject("lidar0");
  lidar0["distance"] = getLidarDistance(0);
  lidar0["engaged"] = isLidarEngaged(0);

  JsonObject lidar1 = report.createNestedObject("lidar1");
  lidar1["distance"] = getLidarDistance(1);
  lidar1["engaged"] = isLidarEngaged(1);

  JsonObject lidar2 = report.createNestedObject("lidar2");
  lidar2["distance"] = getLidarDistance(2);
  lidar2["engaged"] = isLidarEngaged(2);

  JsonObject lidar3 = report.createNestedObject("lidar3");
  lidar3["distance"] = getLidarDistance(3);
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

int Quadrant::_setLidarAddress(uint8_t index) {

  for (int i=0; i<4; i++) {
    setLed(i, LOW);
  }
  delay(100);

  digitalWrite(_ledPins[index], HIGH);
  digitalWrite(_lidarPins[index], HIGH);
  delay(100);

  if(!_lidars[index]->begin(_lidarAddrs[index])) {
    Serial.print(F("Failed to boot VL53L0X "));
    Serial.println(index);
    digitalWrite(_ledPins[index], LOW);
    return 1;
  }

  digitalWrite(_ledPins[index], LOW);

  return 0;

}

void Quadrant::_setLidarContinuous(uint8_t index){

  _lidars[index]->startRangeContinuous();

}

bool Quadrant::_isLidarReady(uint8_t index){

  bool resp;

  resp =_lidars[index]->isRangeComplete();

  if (_lidars[index]->Status != VL53L0X_ERROR_NONE) {
    Serial.print(F("Lidar index "));
    Serial.print(index);
    Serial.print(F(" returned Status = "));
    Serial.println(_lidars[index]->Status);
    return false;
  }

  return resp;

}

int Quadrant::_readLidar(uint8_t index){

  return _lidars[index]->readRange();

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

