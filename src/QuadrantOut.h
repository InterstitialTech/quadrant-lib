#ifndef QuadrantOut_h
#define QuadrantOut_h

#include <ArduinoJson.h>
#include "QuadrantDSP.h"
#include "QuadrantCommon.h"
#include "AD5317R.h"

class QuadrantOut {

  public:

    void begin(void);
    void configureReport(enum QUADRANT_REPORT_FIELD field, bool enabled);
    void updateReport(QuadrantDSP *dsp);
		void reportEvent(const char *event_name);
    void handleMidiThru(void);

    // INPUTS
    bool dip1(void);
    bool dip2(void);

    // OUTPUTS
    void displayStartupLeds(void);
    void setLed(int index, int state);
    void setCV(int chan, float voltage);
    void setGate(int index, int state);
    void sendMidiNoteOn(uint8_t, uint8_t, uint8_t);
    void sendMidiNoteOff(uint8_t, uint8_t);
    void sendMidiControlChange(uint8_t, uint8_t, uint8_t);
    void printReportToSerial(void);

  private:

    AD5317R *_dac;

    const uint8_t _ledPins[4] = {QUADRANT_LED0_PIN, QUADRANT_LED1_PIN, QUADRANT_LED2_PIN, QUADRANT_LED3_PIN};
    const uint8_t _dacAddrs[4] = {AD5317R_DAC_D_ADDR, AD5317R_DAC_C_ADDR, AD5317R_DAC_A_ADDR, AD5317R_DAC_B_ADDR};
    const uint8_t _gatePins[4] = {QUADRANT_GATE0_PIN, QUADRANT_GATE1_PIN, QUADRANT_GATE2_PIN, QUADRANT_GATE3_PIN};

    StaticJsonDocument<512> *_report;
    uint32_t _report_bitmask;

    void _writeDac(uint8_t chan, uint16_t value);

    double _round3(double value);

};


#endif
