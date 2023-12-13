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

}

void QuadrantOut::printReportToSerial(void) {

  _report->clear();

  //JsonObject lidar0 = _report->createNestedObject("lidar0");
  // TODO

  //JsonObject lidar1 = _report->createNestedObject("lidar1");
  // TODO

  //JsonObject lidar2 = _report->createNestedObject("lidar2");
  // TODO

  //JsonObject lidar3 = _report->createNestedObject("lidar3");
  // TODO

  //JsonObject elevation = _report->createNestedObject("elevation");
  // TODO

  //JsonObject pitch = _report->createNestedObject("pitch");
  // TODO

  //JsonObject roll = _report->createNestedObject("roll");
  // TODO

  //JsonObject arc = _report->createNestedObject("arc");
  // TODO

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

