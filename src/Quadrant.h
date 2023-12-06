/*
  Quadrant.h - Library for using Quadrant
  Created by Josh Muffin Gordonson (Interstitial Technology) on July 26 2023
  Released under the CERN OHL 2.0
*/

#ifndef Quadrant_h
#define Quadrant_h

#include "Arduino.h"
#include "Adafruit_VL53L0X.h"
#include <MIDI.h>

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

#define DEFAULT_ENGAGEMENT_THRESHOLD 300

#define QUADRANT_HEIGHT_MM 65
#define QUADRANT_WIDTH_MM 45

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
    void initFilter(uint8_t len);
    void updateFilter(void);

    // setters
    void setEngagementThreshold(int);
    void setLidarEnabled(int index, bool enabled);

    // getters
    float getSampleRate(void);
    bool isLidarEngaged(int index);
    uint16_t getLidarDistance(int index);
    float getLidarDistanceNormalized(int index);
    float getLidarDistanceFiltered(int index);
    bool isElevationEngaged(void);
    float getElevation(void);
    bool isPitchEngaged(void);
    float getPitch(void);
    bool isRollEngaged(void);
    float getRoll(void);
    bool isArcEngaged(void);
    float getArc(void);
    bool isLidarEnabled(int index);

    void printReportToSerial(void);

    // outputs
    void setLed(int index, int state);
    void setCV(int chan, float voltage);
    void sendMidiNoteOn(uint8_t, uint8_t, uint8_t);
    void sendMidiNoteOff(uint8_t, uint8_t);
    void sendMidiControlChange(uint8_t, uint8_t, uint8_t);

  private:

    const uint8_t _ledPins[4] = {LED0_PIN, LED1_PIN, LED2_PIN, LED3_PIN};
    const uint8_t _lidarPins[4]={LIDAR0_ENABLE, LIDAR1_ENABLE, LIDAR2_ENABLE, LIDAR3_ENABLE};
    const uint8_t _lidarAddrs[4] = {LIDAR0_ADDR, LIDAR1_ADDR, LIDAR2_ADDR, LIDAR3_ADDR};
    const uint8_t _dacAddrs[4] = {DAC0_ADDR, DAC1_ADDR, DAC2_ADDR, DAC3_ADDR};

    Adafruit_VL53L0X* _lidars[4];

    uint16_t _distance[4];
    bool _engaged[4];
    bool _lidarEnabled[4];
    unsigned long _tlast, _tnow;
    uint16_t _thresh;
    enum Quadrant_SamplingMode _smode;

    uint16_t *_filter;
    uint8_t _len_filter;
    uint8_t _ifilter;
    bool _filter_enabled;

    void _initLidar(int index);
    void _update_single_pipeline(void);
    void _update_continuous(void);
    bool _isLidarReady(uint8_t index);
    uint16_t _readLidar(uint8_t index);

    void _writeDac(uint8_t chan, int value);

};


#endif
