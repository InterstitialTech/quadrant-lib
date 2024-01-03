#ifndef QuadrantDSP_h
#define QuadrantDSP_h

#include "QuadrantDAQ.h"
#include "QuadrantCommon.h"

class QuadrantDSP {

  public:

    void begin(void);
    void enableFilter(bool enabled);
    void update(QuadrantDAQ *daq);
    void updateFromFifo(void);
    void updateFilter(void);
    void calibrateOffsets(void);
    void setEngagementThreshold(uint16_t value_mm);

    // GETTERS
    uint32_t getTimestamp(void);
    bool isFilterEnabled(void);
    float getLidarDistance(int index);
    float getLidarDistanceNormalized(int index);
    uint16_t getEngagementThreshold(void);
    bool isLidarEngaged(int index);
    bool isElevationEngaged(void);
    float getElevation(void);
    bool isPitchEngaged(void);
    float getPitch(void);
    bool isRollEngaged(void);
    float getRoll(void);
    bool isArcEngaged(void);
    float getArc(void);

  private:

    uint16_t _distance[4];
    bool _engaged[4];
    uint32_t _timestamp;
    uint16_t _thresh;
    void _initFilter(uint8_t len);
    uint16_t *_filter;
    uint8_t _len_filter;
    uint8_t _ifilter;
    bool _filter_enabled;
    uint16_t _offset[4];

};


#endif
