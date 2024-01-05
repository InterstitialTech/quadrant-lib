/*
  Quadrant.h - Library for using Quadrant
  Created by Josh Muffin Gordonson (Interstitial Technology) on July 26 2023
  Released under the CERN OHL 2.0
*/

#ifndef Quadrant_h
#define Quadrant_h

#include "QuadrantDAQ.h"
#include "QuadrantDSP.h"
#include "QuadrantOut.h"

class Quadrant {

  public:

    // setup and loop
    void begin(void);
    void setEngagementThreshold(uint16_t value_mm);
    void enableFilter(bool enabled);
    void configureReport(enum QUADRANT_REPORT_FIELD field, bool enabled);
    void run(void);
		bool newDataReady(void);
    void update(void);

		// getters
		uint16_t getLidarDistance(int index);
		bool isLidarEngaged(int index);
    bool isFilterEnabled(void);
    uint32_t getTimestamp(void);
    uint16_t getEngagementThreshold(void);
    bool isElevationEngaged(void);
    float getElevation(void);
    bool isPitchEngaged(void);
    float getPitch(void);
    bool isRollEngaged(void);
    float getRoll(void);
    bool isArcEngaged(void);
    float getArc(void);
    bool dip1(void);
    bool dip2(void);

		// output
		void setLed(int chan, int state);
		void setCV(int chan, float voltage);
    void setGate(int chan, int state);
    void sendMidiNoteOn(uint8_t note, uint8_t vel, uint8_t chan);
    void sendMidiNoteOff(uint8_t note, uint8_t chan);
    void sendMidiControlChange(uint8_t control_number, uint8_t val, uint8_t chan);
    void handleMidiThru(void);
		void printReportToSerial(void);

    // submodules
    QuadrantDAQ daq;
    QuadrantDSP dsp;
    QuadrantOut out;

};

#endif
