/*
  AD5317R.h
  Copyright Interstitial Technology, PBC (2024)
*/

#ifndef AD5317R_h
#define AD5317R_h

#include "Arduino.h"
#include <SPI.h>

#define AD5317R_DAC_A_ADDR (1 << 0)
#define AD5317R_DAC_B_ADDR (1 << 1)
#define AD5317R_DAC_C_ADDR (1 << 2)
#define AD5317R_DAC_D_ADDR (1 << 3)

class AD5317R {

  public:

    AD5317R(uint8_t pin_nsync, uint8_t pin_sclk, uint8_t pin_sdin, uint8_t pin_nldac);
    void begin(void);
    void setChannelValue(uint8_t chan, uint16_t value);

  private:
    uint8_t _pin_nsync;
    uint8_t _pin_sclk;
    uint8_t _pin_sdin;
    uint8_t _pin_nldac;
    uint8_t _buf[3] = {0};

};

#endif
