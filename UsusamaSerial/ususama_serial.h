#ifndef USUSAMA_SERIAL_H
#define USUSAMA_SERIAL_H
#include "mbed.h"
#include <vector>

#define HEAD_BYTE 0x1D
#define READ_COMMAND 0xFF
#define ESCAPE_BYTE 0x1E
#define ESCAPE_MASK 0x1F

class UsusamaSerial
{
    public:
    UsusamaSerial(PinName tx, PinName rx, int32_t* registar, int baudrate = 115200);
    void set_bufferedserial(int baudrate);
    void set_unbufferedserial(int baudrate);
    void writeData(int32_t data, uint8_t reg);
    uint8_t readData();
    
    int32_t* _register;
    
    private:
    char buffer_w_[64];
    char buffer_r_[64];
    //UnbufferedSerial port_;
    BufferedSerial port_;
};

#endif