#ifndef MBED_I2CMASTER_H
#define MBED_I2CMASTER_H

#include "mbed.h"

class i2c : public I2C
{
    public:
    i2c(PinName p1,PinName p2);
    bool writeSomeData(char addr,char reg, int32_t data ,uint8_t size);
    bool getSlaveRegistarData(char addr, char reg, int32_t *data ,uint8_t size);
};

#endif