#ifndef Quadrant_h
#define Quadrant_h

/*
    Quadrant.h - Library for using Quadrant
    Created by Josh Muffin Gordonson (Interstitial Technology) on July 26 2023
    Released under the CERN OHL 2.0
*/

#include "Arduino.h"
#include "Adafruit_VL53L0X.h"
#include <MIDI.h>

#define LED0 0
#define LED1 23
#define LED2 18
#define LED3 12

#define LOX0_ADDRESS 0x30
#define LOX1_ADDRESS 0x31
#define LOX2_ADDRESS 0x32
#define LOX3_ADDRESS 0x33

#define SHT_LOX0 1
#define SHT_LOX1 24
#define SHT_LOX2 19
#define SHT_LOX3 13


// index 0-3 maps to: led, lidar_enable, and lidar_addr, and dac_addr
// TODO: switch index to NSEW

class Quadrant
{
    public:

        Quadrant();
        void begin();

        int setLidarAddress(uint8_t index, uint8_t addr);
        int readLidar(uint8_t index);

        void setLidarContinuous(uint8_t index);
        void setLidarsContinuous();

        bool checkLidarContinuous(uint8_t index);
        bool checkLidarsContinuous(void);

        int readLidarContinuous(uint8_t index);

        void writeDac(uint8_t dac, int dacData);
        void sendMidiNoteOnOff(uint8_t, uint8_t, uint8_t);
        void sendMidiNoteOnOffRaw(uint8_t, uint8_t, uint8_t);

        const uint8_t leds[4]={LED0, LED1, LED2, LED3};
        void ledsOff();

    private:

        Adafruit_VL53L0X* _loxs[4];
        VL53L0X_RangingMeasurementData_t _measure[4];
        const uint8_t _dacAddress[4]={0x16,0x14,0x12,0x10};
        const uint8_t _lidarAddress[4]={LOX0_ADDRESS, LOX1_ADDRESS, LOX2_ADDRESS, LOX3_ADDRESS};
        const uint8_t _lidarEnable[4]={SHT_LOX0, SHT_LOX1, SHT_LOX2, SHT_LOX3};
        void _disableLidars();

  };

#endif
