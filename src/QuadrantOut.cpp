#include <MIDI.h>
#include "QuadrantOut.h"

SerialPIO QUADRANT_SOFTSERIAL(QUADRANT_MIDI_OUT_PIN, SerialPIO::NOPIN);
MIDI_NAMESPACE::SerialMIDI<SerialPIO> QUADRANT_SOFTSERIAL_MIDI_OUT(QUADRANT_SOFTSERIAL);
MIDI_CREATE_INSTANCE(SerialPIO, QUADRANT_SOFTSERIAL_MIDI_OUT, QUADRANT_MIDI_OUT)

void QuadrantOut::begin(void){

  // LEDS
  for (int i=0; i<4; i++) {
    pinMode(_ledPins[i], OUTPUT); 
    digitalWrite(_ledPins[i], LOW);
  }

  // MIDI in
  Serial2.end();
  Serial2.setRX(QUADRANT_MIDI_IN_PIN); // same as INPUT_PULLDOWN below
  Serial2.begin(31250);

  // MIDI out
  QUADRANT_SOFTSERIAL.setInverted(true,true);
  QUADRANT_MIDI_OUT.begin();

  // init JSON report
  _report = new StaticJsonDocument<512>;;
  _report_bitmask = 0x0;

  // default report config
  configureReport(REPORT_FIELD_TIMESTAMP, true);
  configureReport(REPORT_FIELD_LIDAR0, true);
  configureReport(REPORT_FIELD_LIDAR1, true);
  configureReport(REPORT_FIELD_LIDAR2, true);
  configureReport(REPORT_FIELD_LIDAR3, true);
  configureReport(REPORT_FIELD_ELEVATION, true);
  configureReport(REPORT_FIELD_PITCH, true);
  configureReport(REPORT_FIELD_ROLL, true);
  configureReport(REPORT_FIELD_ARC, true);

  // gate pins
  for (int i=0; i<4; i++) {
    pinMode(_gatePins[i], OUTPUT); 
    digitalWrite(_gatePins[i], LOW);
  }

  // DAC
  _dac = new AD5317R(QUADRANT_DAC_NSYNC_PIN, QUADRANT_DAC_SCLK_PIN,
                      QUADRANT_DAC_SDIN_PIN, QUADRANT_DAC_NLDAC_PIN);
  _dac->begin();

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

// OUTPUTS

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

  QUADRANT_MIDI_OUT.sendNoteOn(note, vel, chan);

}

void QuadrantOut::sendMidiNoteOff(uint8_t note, uint8_t chan){

  QUADRANT_MIDI_OUT.sendNoteOff(note, 0, chan);

}

void QuadrantOut::sendMidiControlChange(uint8_t control_number, uint8_t val, uint8_t chan){

  QUADRANT_MIDI_OUT.sendControlChange(control_number, val, chan);

}

void QuadrantOut::handleMidiThru(void) {

  // simple low-level passthrough

  int c;

  Serial.println("checking...");
  while((Serial2.available() > 0)) {
    Serial.println("\thello input..");
    if ((c = Serial2.read()) >= 0) {
      Serial.println("\t\tforwarding!");
      QUADRANT_SOFTSERIAL.write((uint8_t)c);
    }
  }
  Serial.println("...done checking:");

}

// private methods below

void QuadrantOut::_writeDac(uint8_t chan, uint16_t value){

  if (chan > 3) chan = 3;

  _dac->setChannelValue(_dacAddrs[chan], value);

}

double QuadrantOut::_round3(double value) {

    // convenience function for limiting sig figs in json serialization

   return (int)(value * 1000 + 0.5) / 1000.0;

}

