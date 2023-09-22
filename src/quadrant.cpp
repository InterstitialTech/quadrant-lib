#include <Arduino.h>
#include <Wire.h>
#include "quadrant.h"

SerialPIO softSerial(11, SerialPIO::NOPIN);
MIDI_NAMESPACE::SerialMIDI<SerialPIO> softSerialMidi(softSerial);
MIDI_CREATE_INSTANCE(SerialPIO, softSerialMidi, QMIDI)

Quadrant::Quadrant(){
}

void Quadrant::begin(){

  // LEDS
  pinMode(LED0, OUTPUT); 
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  // Lidars
  pinMode(SHT_LOX0, OUTPUT);
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  _disableLidars();
  for (int i=0; i<4; i++) {
    _loxs[i] = new Adafruit_VL53L0X();
  }
  setLidarAddress(0, LOX0_ADDRESS);
  setLidarAddress(1, LOX1_ADDRESS);
  setLidarAddress(2, LOX2_ADDRESS);
  setLidarAddress(3, LOX3_ADDRESS);

  // DAC
  Wire1.setSDA(6);
  Wire1.setSCL(7);
  Wire1.begin();


  // MIDI
  softSerial.setInverted(true,true);
  QMIDI.begin();

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

void Quadrant::writeDac(uint8_t dac, int dacData){

  if (dacData > 1023){ //limit dacData to 1023
    dacData = 1023;
  }

  Wire1.beginTransmission(byte(0x58));
  Wire1.write(_dacAddress[dac]);
  Wire1.write(dacData>>2);
  Wire1.write((dacData&0x03)<<6);
  Wire1.endTransmission();

}

void Quadrant::_disableLidars(){

  digitalWrite(SHT_LOX0, LOW);    
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

}

void Quadrant::ledsOff(){

  digitalWrite(LED0, LOW);    
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);

}

int Quadrant::setLidarAddress(uint8_t index, uint8_t addr) {

  ledsOff();
  delay(100);

  digitalWrite(leds[index], HIGH);
  digitalWrite(_lidarEnable[index], HIGH);
  delay(100);

  if(!_loxs[index]->begin(addr)) {
    Serial.print(F("Failed to boot VL53L0X "));
    Serial.println(index);
    digitalWrite(leds[index], LOW);
    return 1;
  }

  digitalWrite(leds[index], LOW);

  return 0;

}

void Quadrant::setLidarContinuous(uint8_t index){

  _loxs[index]->startRangeContinuous();

}

void Quadrant::setLidarsContinuous(void){

  for (int i=0; i<4; i++) {
    setLidarContinuous(i);
  }

}

bool Quadrant::checkLidarContinuous(uint8_t index){

  bool resp;

  resp =_loxs[index]->isRangeComplete();

  if (_loxs[index]->Status != VL53L0X_ERROR_NONE) {
    Serial.print(F("Lidar index "));
    Serial.print(index);
    Serial.print(F(" returned Status = "));
    Serial.println(_loxs[index]->Status);
    return false;
  }

  return resp;

}

bool Quadrant::checkLidarsContinuous(void){

  bool ready;

  ready = true;
  for (int i=0; i<4; i++) {
    ready = (ready && checkLidarContinuous(i));
  }

  return ready;

}

int Quadrant::readLidarContinuous(uint8_t index){

  return _loxs[index]->readRange();

}

int Quadrant::readLidar(uint8_t index) {

  digitalWrite(leds[index], HIGH);

  _loxs[index]->rangingTest(&_measure[index], false); // pass in 'true' to get debug data printout!

  digitalWrite(leds[index], LOW);

  if(_measure[index].RangeStatus == 4) {
    Serial.print(F("Out of range: index =  "));
    Serial.println(index);
  }
  
  return _measure[index].RangeMilliMeter;

}

