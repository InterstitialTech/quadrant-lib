#include <Wire.h>

#include "Quadrant.h"
#include "QuadrantCommon.h"

SerialPIO softSerial(11, SerialPIO::NOPIN);
MIDI_NAMESPACE::SerialMIDI<SerialPIO> softSerialMidi(softSerial);
MIDI_CREATE_INSTANCE(SerialPIO, softSerialMidi, QMIDI)

void Quadrant::begin(void){

  Wire.setClock(400000);
  Wire.begin();

  // initialize private variables
  for (int i=0; i<4; i++) {
    _distance[i] = 0xff;
    _engaged[i] = false;
    _lidarEnabled[i] = false;
  }

  _thresh = QUADRANT_THRESH_DEFAULT_MM;
  _smode = QUADRANT_SAMPLINGMODE_CONTINUOUS;

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

  // gate pins
  for (int i=0; i<4; i++) {
    pinMode(_gatePins[i], OUTPUT); 
    digitalWrite(_gatePins[i], LOW);
  }

  // MIDI
  softSerial.setInverted(true,true);
  QMIDI.begin();

  // init JSON report
  _report = new StaticJsonDocument<512>;;

}

void Quadrant::setLidarEnabled(int index, bool enabled) {

  _lidarEnabled[index] = enabled;

}

bool Quadrant::isLidarEnabled(int index) {

  return _lidarEnabled[index];

}

void Quadrant::setEngagementThreshold(uint16_t thresh_mm) {

  _thresh = thresh_mm;

}

void Quadrant::update(void) {

  switch (_smode) {
    case QUADRANT_SAMPLINGMODE_SINGLE_SEQUENTIAL:
      _update_single_sequential();
      break;
    case QUADRANT_SAMPLINGMODE_SINGLE_PIPELINE:
      break;
    case QUADRANT_SAMPLINGMODE_CONTINUOUS:
      _update_continuous_round_robin();
      break;
    case QUADRANT_SAMPLINGMODE_CONTINUOUS_TIMED:
      break;
    default:
      break;
  }

}

void Quadrant::pushFrameMulticore(void) {

    for (int i=0; i<4; i++) {
      rp2040.fifo.push_nb(getLidarDistance(i));
    }

}

// getters

bool Quadrant::isLidarEngaged(int index) {

  return _engaged[index];

}

uint16_t Quadrant::getLidarDistance(int index) {

  // distance in mm

  return _distance[index];

}

// outputs

void Quadrant::printReportToSerial(void) {

  _report->clear();

  JsonObject lidar0 = _report->createNestedObject("lidar0");
  lidar0["engaged"] = isLidarEngaged(0);
  lidar0["distance"] = getLidarDistance(0);

  JsonObject lidar1 = _report->createNestedObject("lidar1");
  lidar1["engaged"] = isLidarEngaged(1);
  lidar1["distance"] = getLidarDistance(1);

  JsonObject lidar2 = _report->createNestedObject("lidar2");
  lidar2["engaged"] = isLidarEngaged(2);
  lidar2["distance"] = getLidarDistance(2);

  JsonObject lidar3 = _report->createNestedObject("lidar3");
  lidar3["engaged"] = isLidarEngaged(3);
  lidar3["distance"] = getLidarDistance(3);

  /*
  JsonObject elevation = _report->createNestedObject("elevation");
  elevation["value"] = _round3(getElevation());
  elevation["engaged"] = isElevationEngaged();

  JsonObject pitch = _report->createNestedObject("pitch");
  pitch["value"] = _round3(getPitch());
  pitch["engaged"] = isPitchEngaged();

  JsonObject roll = _report->createNestedObject("roll");
  roll["value"] = _round3(getRoll());
  roll["engaged"] = isRollEngaged();

  JsonObject arc = _report->createNestedObject("arc");
  arc["value"] = _round3(getArc());
  arc["engaged"] = isArcEngaged();
  */

  serializeJson(*_report, Serial);
  Serial.println();

}

void Quadrant::setLed(int index, int state) {

  digitalWrite(_ledPins[index], state);

}

void Quadrant::setCV(int chan, float voltage) {

  voltage = constrain(voltage, 0.0, 5.0);

  _writeDac(chan, voltage * 1023 / 5);

}

void Quadrant::setGate(int index, int state) {

  digitalWrite(_gatePins[index], state);

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

    if ((_smode == QUADRANT_SAMPLINGMODE_CONTINUOUS)
          || (_smode == QUADRANT_SAMPLINGMODE_CONTINUOUS_TIMED)) {

      _lidars[index]->startContinuous();

    }
    digitalWrite(_ledPins[index], LOW);

}

bool Quadrant::_isLidarReady(uint8_t index){

  return (_lidars[index]->readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07);

}

uint16_t Quadrant::_readLidar(uint8_t index){

  uint16_t d = 0xff;

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled

  d = _lidars[index]->readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);

  _lidars[index]->writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);

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

void Quadrant::_update_single_sequential(void) {

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

void Quadrant::_update_continuous_sequential(void) {

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

void Quadrant::_update_continuous_round_robin(void) {

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

double _round3(double value) {

    // convenience function for limiting sig figs in json serialization

   return (int)(value * 1000 + 0.5) / 1000.0;

}

