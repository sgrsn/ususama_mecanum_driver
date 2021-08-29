#include "ususama_serial.h"

UsusamaSerial::UsusamaSerial(PinName tx, PinName rx, int32_t* registar, int baudrate) : port_(tx, rx)
{
    _register = registar;
    set_bufferedserial(baudrate);
}

void UsusamaSerial::set_bufferedserial(int baudrate)
{
    port_.set_baud(baudrate);
    port_.set_format(
        8,
        BufferedSerial::None,
        1
    );
}
void UsusamaSerial::set_unbufferedserial(int baudrate)
{
    /*port_.baud(baudrate);
    port_.format(
        8,
        SerialBase::None,
        1
    );*/
}

void UsusamaSerial::writeData(int32_t data, uint8_t reg)
{
    uint8_t dataBytes[4] =
    {
        (uint8_t)((data >> 24) & 0xFF),
        (uint8_t)((data >> 16) & 0xFF),
        (uint8_t)((data >>  8) & 0xFF),
        (uint8_t)((data >>  0) & 0xFF)
    };
    
    std::vector<uint8_t> buf;
    buf.push_back(HEAD_BYTE);
    uint8_t checksum = 0;
    buf.push_back(reg);
    checksum += reg;
    for (uint8_t i = 0; i < 4; ++i)
    {
        if ((dataBytes[i] == ESCAPE_BYTE) || (dataBytes[i] == HEAD_BYTE))
        {
            buf.push_back(ESCAPE_BYTE);
            checksum += ESCAPE_BYTE;
            buf.push_back(dataBytes[i] ^ ESCAPE_MASK);
            checksum += dataBytes[i] ^ ESCAPE_MASK;
        }
        else
        {
            buf.push_back(dataBytes[i]);
            checksum += dataBytes[i];
        }
    }

    // 末尾にチェックサムを追加で送信する
    buf.push_back(checksum);
    
    // HEADを抜いたデータサイズ
    uint8_t size = buf.size() - 1;
    
    // HEADの後にsizeを挿入
    buf.insert(buf.begin()+1, size);
    
    for(int i = 0; i < buf.size(); i++)
    {
        buffer_w_[i] = buf[i];
    }
    port_.write(buffer_w_, buf.size());
}

uint8_t UsusamaSerial::readData()
{
    if(!port_.readable()) return 0;
    port_.read(buffer_r_, 1);
    if (buffer_r_[0] != HEAD_BYTE) return 0;

    if(!port_.readable()) return 0;
    port_.read(buffer_r_+1, 1);
    
    // data + 1byte(reg) + 1byte(checksum)
    int size = buffer_r_[1];
    
    if(!port_.readable()) return 0;
    port_.read(buffer_r_+2, size);
    
    uint8_t bytes[8] = {0};
    uint8_t checksum = 0;
    uint8_t reg = buffer_r_[2];
    checksum += reg;
    int index = 3;
    int data_size = size - 2;
    for (int i = 0; i < data_size; ++i)
    {
        uint8_t d = buffer_r_[index++];
        if (d == ESCAPE_BYTE)
        {
            checksum += (uint8_t)d;
            uint8_t nextByte = buffer_r_[index++];
            bytes[i] = (uint8_t)((int)nextByte ^ (int)ESCAPE_MASK);
            checksum += (uint8_t)nextByte;
            data_size--;
        }
        else
        {
            bytes[i] = d;
            checksum += d;
        }
    }
    
    uint8_t checksum_recv = buffer_r_[index++];
    int32_t DATA = 0x00;
    for(int i = 0; i < 4; i++)
    {
        DATA |= (((int32_t)bytes[i]) << (24 - (i*8)));
    }
    if (checksum == checksum_recv)
    {
        *(_register + reg) =  DATA;
        /*for(int i = 0; i < data_size; i++)
            dst_p[i] = bytes[i];*/
        return reg;
    }
    else
    {
        // data error
        return 0;
    }
    return 0;
}