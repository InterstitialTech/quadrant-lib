#include <Arduino.h>
#include <Wire.h>
#include "quadrant.h"

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

  return _distance[index];

}

bool Quadrant::isElevationEngaged(void) {

  return _engaged[0] && _engaged[1] && _engaged[2] && _engaged[3];

}

float Quadrant::getElevation(void) {

    return float(_distance[0] + _distance[1] + _distance[2] + _distance[3]) / 4;

}

bool Quadrant::isPitchEngaged(void) {

  return _engaged[1] && _engaged[3];

}

float Quadrant::getPitch(void) {

    return float(_distance[1] - _distance[3]) / 300;

}

bool Quadrant::isRollEngaged(void) {

  return _engaged[0] && _engaged[2];

}

float Quadrant::getRoll(void) {

    return float(_distance[2] - _distance[0]) / 300;

}

bool Quadrant::isArcEngaged(void) {

  return _engaged[0] && _engaged[1] && _engaged[2] && _engaged[3];

}

float Quadrant::getArc(void) {

    return float(_distance[0] - _distance[1] + _distance[2] - _distance[3]) / 600;

}

// outputs

void Quadrant::setLed(int index, int state) {

  digitalWrite(_ledPins[index], state);

}

void Quadrant::setCV(int chan, float voltage) {

  voltage = constrain(voltage, 0.0, 5.0);

  _writeDac(chan, voltage * 1023 / 5);

}

void Quadrant::sendMidiNoteOnOff(uint8_t note, uint8_t vel, uint8_t chan){

  QMIDI.sendNoteOn(note, vel, chan);
  delay(20);
  QMIDI.sendNoteOff(note, 0, chan);

}

void Quadrant::sendMidiNoteOnOffRaw(uint8_t note, uint8_t vel, uint8_t chan){

  softSerial.write((uint8_t) (0x8 << 4) | (chan & 0b1111));
  softSerial.write((uint8_t) note & 0b1111111);
  softSerial.write((uint8_t) vel & 0b1111111);
  delay(100);
  softSerial.write((uint8_t) (0x9 << 4) | (chan & 0b1111));
  softSerial.write((uint8_t) note & 0b1111111);
  softSerial.write((uint8_t) 0);

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

