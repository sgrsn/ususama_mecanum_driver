#ifndef USUSAMA_PROTOCOL_H
#define USUSAMA_PROTOCOL_H

#include "mecanum_controller.hpp"
//#include <vector>
//#include <optional>

static class UsusamaProtocol
{
    public:
    enum RobotState
    {
        Start=0,
        Wait=1,
        Move=2,
        Reach=3,
        Stop=4,
        Reset=5,
    };
    struct MoveCommand_t
    {
        float x;
        float y;
        float theta;
        bool enable;
    };
    struct MoveReply_t
    {
      float x;
      float y;
      float theta;
    };
    struct StopCommand_t
    {
        bool stop;
    };
    struct StopReply_t
    {
        Pose2D pose;
        bool is_stop;
    };
    
    UsusamaProtocol()
    {
    }
    
    static int encode_float2int(float val)
    {
      return (int)(val * 1000);
    }
    static float decode_int2float(int val)
    {
      return (float)val / 1000;
    }
    
    public:
    static const uint8_t COMMAND_MOVE =        0x04;
    static const uint8_t COMMAND_POSE_X =      0x05;
    static const uint8_t COMMAND_POSE_Y =      0x06;
    static const uint8_t COMMAND_POSE_THETA =  0x07;
    static const uint8_t COMMAND_RESET_ODOMETRY =  0x015;
    
    static const uint8_t REPLY_ROBOT_STATE =   0x03;
    static const uint8_t REPLY_MOVE =          0x04;
    static const uint8_t REPLY_COMMAND_X =        0x05;
    static const uint8_t REPLY_COMMAND_Y =        0x06;
    static const uint8_t REPLY_COMMAND_THETA =    0x07;
    static const uint8_t REPLY_STATE_X = 0x011;
    static const uint8_t REPLY_STATE_Y = 0x012;
    static const uint8_t REPLY_STATE_THETA = 0x13;
    static const uint8_t REPLY_RESET_ODOMETRY =  0x15;
    
    static const uint8_t DEBUG_CONSOLE = 0x20;
};

#endif