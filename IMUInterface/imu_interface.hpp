#ifndef IMU_INTERFACE_HPP
#define IMU_INTERFACE_HPP

#include "PMSU_100.hpp"

class IMUInterface
{
    public:
    IMUInterface()
    {
    }
    virtual double GetYawRadians(){return 0;}
};

class PMSUInterface : public IMUInterface
{
    public:
    PMSUInterface(PinName tx, PinName rx) : IMUInterface(), device_(tx, rx)
    {
    }
    double GetYawRadians()
    {
        device_.update();
        return device_.yaw_rad;
    }
    
    private:
    PMSUSerial device_;
};


#endif