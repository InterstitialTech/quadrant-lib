#include <Wire.h>

#include "Quadrant.h"

SerialPIO softSerial(11, SerialPIO::NOPIN);
MIDI_NAMESPACE::SerialMIDI<SerialPIO> softSerialMidi(softSerial);
MIDI_CREATE_INSTANCE(SerialPIO, softSerialMidi, QMIDI)

void Quadrant::begin(void){

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
  softSerial.setInverted(true,true);
  QMIDI.begin();

  // init JSON report
  _report = new StaticJsonDocument<512>;;

}

// outputs

void Quadrant::printReportToSerial(void) {

  _report->clear();

  JsonObject lidar0 = _report->createNestedObject("lidar0");
  lidar0["engaged"] = daq.isLidarEngaged(0);
  lidar0["distance"] = daq.getLidarDistance(0);

  JsonObject lidar1 = _report->createNestedObject("lidar1");
  lidar1["engaged"] = daq.isLidarEngaged(1);
  lidar1["distance"] = daq.getLidarDistance(1);

  JsonObject lidar2 = _report->createNestedObject("lidar2");
  lidar2["engaged"] = daq.isLidarEngaged(2);
  lidar2["distance"] = daq.getLidarDistance(2);

  JsonObject lidar3 = _report->createNestedObject("lidar3");
  lidar3["engaged"] = daq.isLidarEngaged(3);
  lidar3["distance"] = daq.getLidarDistance(3);

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

double _round3(double value) {

    // convenience function for limiting sig figs in json serialization

   return (int)(value * 1000 + 0.5) / 1000.0;

}

