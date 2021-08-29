#ifndef PMSU_100_HPP
#define PMSU_100_HPP

//#include "BufferedSerial.h"
#include "mbed.h"

// Maximum number of element the application buffer can contain
#define PMSU_MAXIMUM_BUFFER_SIZE     48
#define PMSU_BAUDRATE           1041666

const float YAW_INVERSE = -1.0;
const double PI = 3.14159265359;

const float omega_reverse_threashold = 200;

struct ACCELERRATION {
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
};

struct GYRO {
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
};

struct QUATERNION {
  float w = 0;
  float x = 0;
  float y = 0;
  float z = 0;
};

struct PMSU
{
    uint8_t counter = 0;
    ACCELERRATION acc;
    GYRO gyro;
    QUATERNION q;
    int16_t yaw_raw = 0;
};

class PMSUSerial : public BufferedSerial
{
    public:
    PMSUSerial(PinName tx, PinName rx) : port_(tx, rx), BufferedSerial(tx, rx)
    {
        init();
    }
    
    void init()
    {
        port_.set_baud(PMSU_BAUDRATE);
        port_.set_format(
            /* bits */ 8,
            /* parity */ BufferedSerial::Odd,
            /* stop bit */ 1
        );
    }
    void update()
    {
        imu_read();
        computeIMU();
    }
    
    private:
    void imu_read()
    {
        uint32_t num = port_.read(buf_, 4);
        if(buf_[0] == 0x99 && buf_[1] == 0x66 && buf_[2] == 0x66 && buf_[3] == 0x99)
        {
            num = port_.read(buf_, 44);
            imu_.counter = buf_[3];
            imu_.acc.x = buf_[5] | buf_[4]<<8;
            imu_.acc.y = buf_[7] | buf_[6]<<8;
            imu_.acc.z = buf_[9] | buf_[8]<<8;
            imu_.gyro.x = buf_[11] | buf_[10]<<8;
            imu_.gyro.y = buf_[13] | buf_[12]<<8;
            imu_.gyro.z = buf_[15] | buf_[14]<<8;
            imu_.yaw_raw = buf_[41] | buf_[40]<<8;
        }
    }
    
    void computeIMU()
    {
        yaw_deg = (float)imu_.yaw_raw / 100 * YAW_INVERSE;
        checkReverseAngle();
        yaw_deg_computed = yaw_deg + 360.0 * (float)rev_count;
        yaw_rad = (float)yaw_deg_computed  * PI / 180;
    }

    void checkReverseAngle()
    {
        if((yaw_deg - last_yaw_deg) > omega_reverse_threashold )
        {
            rev_count--;
        }        
        else if((yaw_deg - last_yaw_deg) < -omega_reverse_threashold )
        {
            rev_count++;
        }
        last_yaw_deg = yaw_deg;
    }
    
    public:
    float yaw_deg;
    float yaw_rad;

    float yaw_deg_computed = 0;
    int rev_count = 0;
    
    private:
    BufferedSerial port_;
    char buf_[PMSU_MAXIMUM_BUFFER_SIZE] = {0};
    PMSU imu_;
    float last_yaw_deg = 0;
};

#endif