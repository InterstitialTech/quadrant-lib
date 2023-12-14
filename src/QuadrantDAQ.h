#ifndef QuadrantDAQ_h
#define QuadrantDAQ_h

#include <VL53L0X.h>

#include "QuadrantCommon.h"


class QuadrantDAQ {

  public:

    enum SamplingMode {
      SAMPLINGMODE_SINGLE_SEQUENTIAL,
      SAMPLINGMODE_SINGLE_PIPELINE,
      SAMPLINGMODE_CONTINUOUS,
      SAMPLINGMODE_CONTINUOUS_TIMED
    }; 

    void begin(void);
    void update(void);
    void pushToFifo(void);

    // setters
    void setLidarEnabled(int index, bool enabled);

    // getters
    uint16_t getLidarDistance(int index);
    uint32_t getTimestamp(void);
    bool isLidarEngaged(int index);
    bool isLidarEnabled(int index);

  private:

    const uint8_t _lidarPins[4] = {QUADRANT_LIDAR0_ENABLE, QUADRANT_LIDAR1_ENABLE, QUADRANT_LIDAR2_ENABLE, QUADRANT_LIDAR3_ENABLE};
    const uint8_t _lidarAddrs[4] = {QUADRANT_LIDAR0_ADDR, QUADRANT_LIDAR1_ADDR, QUADRANT_LIDAR2_ADDR, QUADRANT_LIDAR3_ADDR};

    VL53L0X* _lidars[4];

    uint16_t _distance[4];
    uint32_t _timestamp;
    bool _engaged[4];
    bool _lidarEnabled[4];
    enum SamplingMode _smode;

    void _initLidar(int index);
    void _update_single_sequential(void);
    void _update_continuous_sequential(void);
    void _update_continuous_round_robin(void);
    bool _isLidarReady(uint8_t index);
    uint16_t _readLidar(uint8_t index);

};


#endif