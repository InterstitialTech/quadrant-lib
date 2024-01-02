/*
  AD5317R.h
  Copyright Interstitial Technology, PBC (2024)
*/

#include "AD5317R.h"

AD5317R::AD5317R(uint8_t pin_nsync, uint8_t pin_sclk, uint8_t pin_sdin, uint8_t pin_nldac) {
  _pin_nsync = pin_nsync;
  _pin_sclk = pin_sclk;
  _pin_sdin = pin_sdin;
  _pin_nldac = pin_nldac;
}

void AD5317R::begin(void) {

  // set up SPI1
  SPI1.setSCK(_pin_sclk);
  SPI1.setTX(_pin_sdin);
  pinMode(_pin_nsync, OUTPUT);
  digitalWrite(_pin_nsync, HIGH);
  SPI1.begin(false);
  SPI1.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1)); 

  // power up
  _buf[0] = (0b0100 << 4);
  _buf[1] = 0;
  _buf[2] = 0;
	digitalWrite(_pin_nsync, LOW);
  SPI1.transfer(_buf, 3);
	digitalWrite(_pin_nsync, HIGH);

  // software reset
  _buf[0] = (0b0110 << 4);
  _buf[1] = 0;
  _buf[2] = 0;
	digitalWrite(_pin_nsync, LOW);
  SPI1.transfer(_buf, 3);
	digitalWrite(_pin_nsync, HIGH);

  // internal reference setup
  _buf[0] = (0b0111 << 4);
  _buf[1] = 0;
  _buf[2] = 0;
	digitalWrite(_pin_nsync, LOW);
  SPI1.transfer(_buf, 3);
	digitalWrite(_pin_nsync, HIGH);

  // NLDAC low (instantaneous updates)
  pinMode(_pin_nldac, OUTPUT);
  digitalWrite(_pin_nldac, LOW);

}

void AD5317R::setChannelValue(uint8_t chan_addr, uint16_t value) {

  _buf[0] = (0b0011 << 4) | (chan_addr & 0b1111);
  _buf[1] = (value >> 2) & 0xff;
  _buf[2] = (value << 6) & 0xff;

	digitalWrite(_pin_nsync, LOW);
  SPI1.transfer(_buf, 3);
	digitalWrite(_pin_nsync, HIGH);

}

