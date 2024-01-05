#ifndef QuadrantCommon_h
#define QuadrantCommon_h

#include "Arduino.h"

#define QUADRANT_LIDAR0_ENABLE 1
#define QUADRANT_LIDAR1_ENABLE 24
#define QUADRANT_LIDAR2_ENABLE 17
#define QUADRANT_LIDAR3_ENABLE 14

#define QUADRANT_LIDAR0_GPIO 2
#define QUADRANT_LIDAR1_GPIO 26
#define QUADRANT_LIDAR2_GPIO 18
#define QUADRANT_LIDAR3_GPIO 15

#define QUADRANT_LIDAR0_ADDR 0x30
#define QUADRANT_LIDAR1_ADDR 0x31
#define QUADRANT_LIDAR2_ADDR 0x32
#define QUADRANT_LIDAR3_ADDR 0x33

#define QUADRANT_LED0_PIN 0
#define QUADRANT_LED1_PIN 23
#define QUADRANT_LED2_PIN 16
#define QUADRANT_LED3_PIN 13

#define QUADRANT_DAC_NLDAC_PIN 8
#define QUADRANT_DAC_NSYNC_PIN 9
#define QUADRANT_DAC_SCLK_PIN 10
#define QUADRANT_DAC_SDIN_PIN 11

#define QUADRANT_GATE0_PIN 19
#define QUADRANT_GATE1_PIN 20
#define QUADRANT_GATE2_PIN 22
#define QUADRANT_GATE3_PIN 21

#define QUADRANT_MIDI_OUT_PIN 12
#define QUADRANT_MIDI_IN_PIN 25

#define QUADRANT_DIP1_PIN 3
#define QUADRANT_DIP2_PIN 6

#define QUADRANT_THRESH_DEFAULT_MM 500
#define QUADRANT_FILTER_LENGTH_DEFAULT 4
#define QUADRANT_TIMEOUT_MS 50
#define QUADRANT_HEIGHT_MM 65
#define QUADRANT_WIDTH_MM 45

enum QUADRANT_REPORT_FIELD{
  QUADRANT_REPORT_FIELD_TIMESTAMP= 0,
  QUADRANT_REPORT_FIELD_LIDAR0 = 1,
  QUADRANT_REPORT_FIELD_LIDAR1 = 2,
  QUADRANT_REPORT_FIELD_LIDAR2 = 3,
  QUADRANT_REPORT_FIELD_LIDAR3 = 4,
  QUADRANT_REPORT_FIELD_ELEVATION = 5,
  QUADRANT_REPORT_FIELD_PITCH = 6,
  QUADRANT_REPORT_FIELD_ROLL = 7,
  QUADRANT_REPORT_FIELD_ARC = 8
};

#endif
