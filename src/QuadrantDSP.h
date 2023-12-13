#ifndef QuadrantDSP_h
#define QuadrantDSP_h

#include "QuadrantCommon.h"

class QuadrantDSP {

  public:
    void begin(void);
    void update(uint16_t *frame);
    void popFrame(uint16_t *frame);
    void initFilter(uint8_t len);
    void updateFilter(void);
    void calibrateOffsets(void);
    float getLidarDistanceFiltered(int index);
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
    uint16_t _thresh;
    uint16_t *_filter;
    uint8_t _len_filter;
    uint8_t _ifilter;
    bool _filter_enabled;
    uint16_t _offset[4];

};


#endif
