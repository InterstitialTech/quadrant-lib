#include <Wire.h>
#include <MIDI.h>
#include "QuadrantOut.h"

SerialPIO QUADRANT_SOFTSERIAL(11, SerialPIO::NOPIN);
MIDI_NAMESPACE::SerialMIDI<SerialPIO> QUADRANT_SOFTSERIALMIDI(QUADRANT_SOFTSERIAL);
MIDI_CREATE_INSTANCE(SerialPIO, QUADRANT_SOFTSERIALMIDI, QUADRANT_MIDI)


void QuadrantOut::begin(void){

  // LEDS
  for (int i=0; i<4; i++) {
    pinMode(_ledPins[i], OUTPUT); 
    digitalWrite(_ledPins[i], LOW);
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
  QUADRANT_SOFTSERIAL.setInverted(true,true);
  QUADRANT_MIDI.begin();

  // init JSON report
  _report = new StaticJsonDocument<512>;;
  _report_bitmask = 0x0;

  configureReport(REPORT_FIELD_TIMESTAMP, true);
  configureReport(REPORT_FIELD_LIDAR0, true);
  configureReport(REPORT_FIELD_LIDAR1, true);
  configureReport(REPORT_FIELD_LIDAR2, true);
  configureReport(REPORT_FIELD_LIDAR3, true);

}

void QuadrantOut::displayStartupLeds(void) {

  // run a sequence of LED flashes to indicate the device has started

  for (int i=0; i<4; i++) {
    setLed(i, LOW);
  }

  for (int i=0; i<8; i++) {
    setLed(i%4, HIGH);
    delay(100);
    setLed(i%4, LOW);
  }

}

void QuadrantOut::configureReport(enum REPORT_FIELD field, bool enabled) {

  if (enabled) {
    _report_bitmask |=  (1 << field);
  } else {
    _report_bitmask &= ~(1 << field);
  }

}

void QuadrantOut::updateReport(QuadrantDSP *dsp) {

  JsonObject jsonObject;
  _report->clear();

  if (_report_bitmask & (1 << REPORT_FIELD_TIMESTAMP)) {
    jsonObject = _report->to<JsonObject>();
    jsonObject["ts"] = dsp->getTimestamp();
  }

  if (_report_bitmask & (1 << REPORT_FIELD_LIDAR0)) {
    jsonObject = _report->createNestedObject("l0");
    jsonObject["en"] = dsp->isLidarEngaged(0);
    jsonObject["dist"] = dsp->isFilterEnabled() ? _round3(dsp->getLidarDistanceFiltered(0)) : dsp->getLidarDistance(0);
  }

  if (_report_bitmask & (1 << REPORT_FIELD_LIDAR1)) {
    jsonObject = _report->createNestedObject("l1");
    jsonObject["en"] = dsp->isLidarEngaged(1);
    jsonObject["dist"] = dsp->isFilterEnabled() ? _round3(dsp->getLidarDistanceFiltered(1)) : dsp->getLidarDistance(1);
  }

  if (_report_bitmask & (1 << REPORT_FIELD_LIDAR2)) {
    jsonObject = _report->createNestedObject("l2");
    jsonObject["en"] = dsp->isLidarEngaged(2);
    jsonObject["dist"] = dsp->isFilterEnabled() ? _round3(dsp->getLidarDistanceFiltered(2)) : dsp->getLidarDistance(2);
  }

  if (_report_bitmask & (1 << REPORT_FIELD_LIDAR3)) {
    jsonObject = _report->createNestedObject("l3");
    jsonObject["en"] = dsp->isLidarEngaged(3);
    jsonObject["dist"] = dsp->isFilterEnabled() ? _round3(dsp->getLidarDistanceFiltered(3)) : dsp->getLidarDistance(3);
  }

  if (_report_bitmask & (1 << REPORT_FIELD_ELEVATION)) {
    jsonObject = _report->createNestedObject("elevation");
    jsonObject["en"] = dsp->isElevationEngaged();
    jsonObject["val"] = _round3(dsp->getElevation());
  }

  if (_report_bitmask & (1 << REPORT_FIELD_PITCH)) {
    jsonObject = _report->createNestedObject("pitch");
    jsonObject["en"] = dsp->isPitchEngaged();
    jsonObject["val"] = _round3(dsp->getPitch());
  }

  if (_report_bitmask & (1 << REPORT_FIELD_ROLL)) {
    jsonObject = _report->createNestedObject("roll");
    jsonObject["en"] = dsp->isRollEngaged();
    jsonObject["val"] = _round3(dsp->getRoll());
  }

  if (_report_bitmask & (1 << REPORT_FIELD_ARC)) {
    jsonObject = _report->createNestedObject("arc");
    jsonObject["en"] = dsp->isArcEngaged();
    jsonObject["val"] = _round3(dsp->getArc());
  }

}

void QuadrantOut::printReportToSerial(void) {

  serializeJson(*_report, Serial);
  Serial.println();

}

void QuadrantOut::setLed(int index, int state) {

  digitalWrite(_ledPins[index], state);

}

void QuadrantOut::setCV(int chan, float voltage) {

  voltage = constrain(voltage, 0.0, 5.0);

  _writeDac(chan, voltage * 1023 / 5);

}

void QuadrantOut::setGate(int index, int state) {

  digitalWrite(_gatePins[index], state);

}

void QuadrantOut::sendMidiNoteOn(uint8_t note, uint8_t vel, uint8_t chan){

  QUADRANT_MIDI.sendNoteOn(note, vel, chan);

}

void QuadrantOut::sendMidiNoteOff(uint8_t note, uint8_t chan){

  QUADRANT_MIDI.sendNoteOff(note, 0, chan);

}

void QuadrantOut::sendMidiControlChange(uint8_t control_number, uint8_t val, uint8_t chan){

  QUADRANT_MIDI.sendControlChange(control_number, val, chan);

}

// private methods below

void QuadrantOut::_writeDac(uint8_t chan, int value){

  if (value > 1023) {
    value = 1023;
  }

  Wire1.beginTransmission(byte(0x58));
  Wire1.write(_dacAddrs[chan]);
  Wire1.write(value >> 2);
  Wire1.write((value & 0x03) << 6);
  Wire1.endTransmission();

}

double QuadrantOut::_round3(double value) {

    // convenience function for limiting sig figs in json serialization

   return (int)(value * 1000 + 0.5) / 1000.0;

}

