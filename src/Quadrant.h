/*
  Quadrant.h - Library for using Quadrant
  Created by Josh Muffin Gordonson (Interstitial Technology) on July 26 2023
  Released under the CERN OHL 2.0
*/

#ifndef Quadrant_h
#define Quadrant_h

#include "Arduino.h"
#include <VL53L0X.h>
#include <MIDI.h>
#include <ArduinoJson.h>

#include "QuadrantDSP.h"

#define LED0_PIN 0
#define LED1_PIN 23
#define LED2_PIN 18
#define LED3_PIN 12

#define LIDAR0_ENABLE 1
#define LIDAR1_ENABLE 24
#define LIDAR2_ENABLE 19
#define LIDAR3_ENABLE 13

#define LIDAR0_ADDR 0x30
#define LIDAR1_ADDR 0x31
#define LIDAR2_ADDR 0x32
#define LIDAR3_ADDR 0x33

#define DAC0_ADDR 0x16
#define DAC1_ADDR 0x14
#define DAC2_ADDR 0x12
#define DAC3_ADDR 0x10

#define GATE0_PIN 2
#define GATE1_PIN 25
#define GATE2_PIN 20
#define GATE3_PIN 14

#define QUADRANT_TIMEOUT_MS 1000


enum Quadrant_SamplingMode {
  QUADRANT_SAMPLINGMODE_SINGLE_SEQUENTIAL,
  QUADRANT_SAMPLINGMODE_SINGLE_PIPELINE,
  QUADRANT_SAMPLINGMODE_CONTINUOUS,
  QUADRANT_SAMPLINGMODE_CONTINUOUS_TIMED
}; 


class Quadrant {

  public:

    void begin(void);
    void update(void);
    void pushFrameMulticore(void);

    // setters
    void setEngagementThreshold(uint16_t thresh_mm);
    void setLidarEnabled(int index, bool enabled);

    // getters
    bool isLidarEngaged(int index);
    uint16_t getLidarDistance(int index);
    bool isLidarEnabled(int index);

    // DSP
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

    const uint8_t _ledPins[4] = {LED0_PIN, LED1_PIN, LED2_PIN, LED3_PIN};
    const uint8_t _lidarPins[4]={LIDAR0_ENABLE, LIDAR1_ENABLE, LIDAR2_ENABLE, LIDAR3_ENABLE};
    const uint8_t _lidarAddrs[4] = {LIDAR0_ADDR, LIDAR1_ADDR, LIDAR2_ADDR, LIDAR3_ADDR};
    const uint8_t _dacAddrs[4] = {DAC0_ADDR, DAC1_ADDR, DAC2_ADDR, DAC3_ADDR};
    const uint8_t _gatePins[4] = {GATE0_PIN, GATE1_PIN, GATE2_PIN, GATE3_PIN};

    VL53L0X* _lidars[4];

    uint16_t _distance[4];
    bool _engaged[4];
    bool _lidarEnabled[4];
    uint16_t _thresh;
    enum Quadrant_SamplingMode _smode;

    void _initLidar(int index);
    void _update_single_sequential(void);
    void _update_continuous_sequential(void);
    void _update_continuous_round_robin(void);
    bool _isLidarReady(uint8_t index);
    uint16_t _readLidar(uint8_t index);

    // output
    StaticJsonDocument<512> *_report;
    void _writeDac(uint8_t chan, int value);
    double _round3(double value);

};


#endif
