/*
  Quadrant.h - Library for using Quadrant
  Created by Josh Muffin Gordonson (Interstitial Technology) on July 26 2023
  Released under the CERN OHL 2.0
*/

#ifndef Quadrant_h
#define Quadrant_h

#include "Arduino.h"
#include <MIDI.h>
#include <ArduinoJson.h>

#include "QuadrantDAQ.h"
#include "QuadrantDSP.h"


class Quadrant {

  public:

    void begin(void);

    // Modules
    QuadrantDAQ daq;
    QuadrantDSP dsp;

    // outputs
    void setLed(int index, int state);
    void setCV(int chan, float voltage);
    void setGate(int index, int state);
    void sendMidiNoteOn(uint8_t, uint8_t, uint8_t);
    void sendMidiNoteOff(uint8_t, uint8_t);
    void sendMidiControlChange(uint8_t, uint8_t, uint8_t);
    void printReportToSerial(void);

  private:

    const uint8_t _ledPins[4] = {QUADRANT_LED0_PIN, QUADRANT_LED1_PIN, QUADRANT_LED2_PIN, QUADRANT_LED3_PIN};
    const uint8_t _dacAddrs[4] = {QUADRANT_DAC0_ADDR, QUADRANT_DAC1_ADDR, QUADRANT_DAC2_ADDR, QUADRANT_DAC3_ADDR};
    const uint8_t _gatePins[4] = {QUADRANT_GATE0_PIN, QUADRANT_GATE1_PIN, QUADRANT_GATE2_PIN, QUADRANT_GATE3_PIN};

    // output
    StaticJsonDocument<512> *_report;
    void _writeDac(uint8_t chan, int value);
    double _round3(double value);

};


#endif
