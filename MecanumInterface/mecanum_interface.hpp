#ifndef MECANUM_INTERFACE_HPP
#define MECANUM_INTERFACE_HPP

#include "i2cmaster.h"

#define MOTOR_NUM 4
#define IIC_ADDR1        0xB0
#define IIC_ADDR2        0xC0
#define IIC_ADDR3        0xD0
#define IIC_ADDR4        0xE0

// AR46SAK-PS50-1 
// Experimentally determined maximum frequency
#define MaxFrequency   25000
#define MinFrequency   0

typedef enum
{
    COAST   = 0,
    BRAKE   = 1,    
    CW      = 2,
    CCW     = 3
}MotorState;

#define WHO_AM_I        0x00
#define MY_IIC_ADDR     0x01
#define MOTOR_DIR       0x04
#define PWM_FREQUENCY   0x05

class MecanumInterface
{
    public:
    MecanumInterface()
    {
    }
    
    int ControlMotor(int ch, int frequency)
    {
        int dir = COAST;
        int cmd_frequency = 0;
        if(ch < 0 || ch > 3)
        {
            //channel error
            return 0;
        }
        else
        {
            if(frequency > 0)
            {
                dir = CW;
                cmd_frequency = frequency;
            }
            else if(frequency < 0)
            {
                dir = CCW;
                cmd_frequency = -frequency;
            }
            else
            {
                dir = BRAKE;
            }
            // 周波数制限 脱調を防ぐ
            if(cmd_frequency > MaxFrequency) 
            {
                cmd_frequency = MaxFrequency;
                if(dir == CW) frequency = MaxFrequency;
                else if(dir == CCW) frequency = -MaxFrequency;
            }
            //else if(cmd_frequency < MinFrequency) cmd_frequency = MinFrequency;
            
            CommandMotor(ch, dir, cmd_frequency);
            
            return frequency;
        }
    }
    
    virtual void CommandMotor(int ch, int dir, int frequency)
    {
    }
};

class MecanumI2C : public MecanumInterface
{
    public:
    MecanumI2C(i2c *device)
    {
        myi2c_ = device;
    }
    
    void CommandMotor(int ch, int dir, int frequency)
    {
        int size = 4;
        int addr[MOTOR_NUM] = {IIC_ADDR1, IIC_ADDR2, IIC_ADDR3, IIC_ADDR4};
        myi2c_ -> writeSomeData(addr[ch], PWM_FREQUENCY, frequency, size);
        myi2c_ -> writeSomeData(addr[ch], MOTOR_DIR, dir, size);
    }
    
    private:
    i2c* myi2c_;
};

#endif