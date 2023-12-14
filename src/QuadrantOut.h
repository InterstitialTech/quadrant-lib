#ifndef QuadrantOut_h
#define QuadrantOut_h

#include <ArduinoJson.h>
#include "QuadrantDSP.h"
#include "QuadrantCommon.h"


class QuadrantOut {

  public:

    void begin(void);
    void displayStartupLeds(void);
    void setLed(int index, int state);
    void setCV(int chan, float voltage);
    void setGate(int index, int state);
    void sendMidiNoteOn(uint8_t, uint8_t, uint8_t);
    void sendMidiNoteOff(uint8_t, uint8_t);
    void sendMidiControlChange(uint8_t, uint8_t, uint8_t);

    enum REPORT_FIELD{
      REPORT_FIELD_TIMESTAMP= 0,
      REPORT_FIELD_LIDAR0 = 1,
      REPORT_FIELD_LIDAR1 = 2,
      REPORT_FIELD_LIDAR2 = 3,
      REPORT_FIELD_LIDAR3 = 4,
      REPORT_FIELD_ELEVATION = 5,
      REPORT_FIELD_PITCH = 6,
      REPORT_FIELD_ROLL = 7,
      REPORT_FIELD_ARC = 8
    };

    void configureReport(enum REPORT_FIELD, bool enabled);
    void updateReport(QuadrantDSP *dsp);
    void printReportToSerial(void);

  private:

    const uint8_t _ledPins[4] = {QUADRANT_LED0_PIN, QUADRANT_LED1_PIN, QUADRANT_LED2_PIN, QUADRANT_LED3_PIN};
    const uint8_t _dacAddrs[4] = {QUADRANT_DAC0_ADDR, QUADRANT_DAC1_ADDR, QUADRANT_DAC2_ADDR, QUADRANT_DAC3_ADDR};
    const uint8_t _gatePins[4] = {QUADRANT_GATE0_PIN, QUADRANT_GATE1_PIN, QUADRANT_GATE2_PIN, QUADRANT_GATE3_PIN};

    StaticJsonDocument<512> *_report;
    uint32_t _report_bitmask;

    void _writeDac(uint8_t chan, int value);

    double _round3(double value);

};


#endif
