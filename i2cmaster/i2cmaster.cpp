#include "i2cmaster.h"

/*example************************************************

#include "mbed.h"
#include "i2cmaster.h"

int main()
{
    i2c master(p28, p27);
    int addr = 0xa0;
    int reg = 0;
    int32_t data;
    int size = 4;
    while(1)
    {
        printf("%d\r\n", data);
        master.writeSomeData(addr, reg, 2345, size);
        master.getSlaveRegistarData(addr, 1, &data, size);
        wait_ms(100);
    }
}


*********************************************************/


i2c::i2c(PinName p1,PinName p2) : I2C(p1,p2)
{
    frequency(400000);
}
bool i2c::writeSomeData(char addr, char reg, int32_t data ,uint8_t size)
{
    char tmp[size];
    for(int i = 0; i < size; i++)
    {
        tmp[i] = (data >> (i*8)) & 0xFF;
    }
    char DATA[2] = {reg, size};
    bool N = I2C::write(addr, DATA, 2);
    N|= I2C::write(addr, tmp, size);
    return N;
}
bool i2c::getSlaveRegistarData(char addr, char reg, int32_t *data ,uint8_t size)
{
    char DATA[2] = {reg,size};
    bool N = I2C::write(addr,DATA,2);
    char _data[size];
    N|= I2C::read(addr, _data, size);
    int32_t tmp = 0;
    for(int i = 0; i < size; i++)
    {
        tmp |= _data[i] << (i*8);
    }
    *data = tmp;
    return N;
}

