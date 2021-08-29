#ifndef USUSAMA_CONTROLLER_H
#define USUSAMA_CONTROLLER_H

#include "ususama_protocol.hpp"
#include "ususama_serial.h"

class UsusamaController
{
    public:
    
    struct UsusamaProtocol::MoveCommand_t move_command;
    struct UsusamaProtocol::MoveReply_t move_reply;

    bool reset_odometry_requested = false;
    bool is_have_next_command = false;
    
    UsusamaController(UsusamaSerial *pc)
    {
        pc_ = pc;
        move_command.enable = false;
    }
    
    void update(float x, float y, float theta)
    {
        switch(last_command_)
        {
            case UsusamaProtocol::COMMAND_POSE_X:
            case UsusamaProtocol::COMMAND_POSE_Y:
            case UsusamaProtocol::COMMAND_POSE_THETA:
                cmd_pose_changed_ = true;
                break;
            case UsusamaProtocol::COMMAND_MOVE:
                break;    
        }
        move_reply.x = x;
        move_reply.y = y;
        move_reply.theta = theta;
    }
    
    void write_robot_state()
    {
        // 送られてきたコマンドによって何を送るか決める
        switch(last_command_)
        {
            case UsusamaProtocol::COMMAND_MOVE:
                reply_current_pose();
                break;
            
            case UsusamaProtocol::COMMAND_POSE_X:
            case UsusamaProtocol::COMMAND_POSE_Y:
            case UsusamaProtocol::COMMAND_POSE_THETA:
                reply_commad_pose();
                break;

            case UsusamaProtocol::COMMAND_RESET_ODOMETRY:
                break;
        }
        pc_ -> writeData(robot_state_reply, UsusamaProtocol::REPLY_ROBOT_STATE);
        
    }
    
    void reply_commad_pose()
    {
        // 受け取った値をそのまま返す
        switch(n_reply_commad_pose)
        {
            case 0:
                pc_ -> writeData(pc_->_register[UsusamaProtocol::COMMAND_POSE_X], UsusamaProtocol::REPLY_COMMAND_X);
                break;
            case 1:
                pc_ -> writeData(pc_->_register[UsusamaProtocol::COMMAND_POSE_Y], UsusamaProtocol::REPLY_COMMAND_Y);
                break;
            case 2:
                pc_ -> writeData(pc_->_register[UsusamaProtocol::COMMAND_POSE_THETA], UsusamaProtocol::REPLY_COMMAND_THETA);
                break;
        }
        n_reply_commad_pose++;
        n_reply_commad_pose = n_reply_commad_pose%3;
    }
    
    void reply_current_pose()
    {
        switch(n_reply_current_pose)
        {
            case 0:
                pc_ -> writeData(UsusamaProtocol::encode_float2int(move_reply.x), UsusamaProtocol::REPLY_STATE_X);
                break;
            case 1:
                pc_ -> writeData(UsusamaProtocol::encode_float2int(move_reply.y), UsusamaProtocol::REPLY_STATE_Y);
                break;
            case 2:
                pc_ -> writeData(UsusamaProtocol::encode_float2int(move_reply.theta), UsusamaProtocol::REPLY_STATE_THETA);
                break;
        }
        n_reply_current_pose++;
        n_reply_current_pose = n_reply_current_pose % 3;
    }
    
    bool is_enable_to_move()
    {
        return move_command.enable;
    }
    bool is_ref_pose_changed()
    {
        return cmd_pose_changed_;
    }
    void notify_use_ref_pose()
    {
        cmd_pose_changed_ = false;
    }
    bool is_reset_called()
    {
        return robot_state_reply == UsusamaProtocol::RobotState::Reset;
    }
    void notify_reset_odometry()
    {
        robot_state_reply = UsusamaProtocol::RobotState::Start;
    }
    
    UsusamaProtocol::MoveCommand_t getMoveCommand()
    {
        return move_command;
    }
    
    void notify_reached_goal()
    {
        move_command.enable = false;
        robot_state_reply = UsusamaProtocol::RobotState::Reach;
    }
    
    void receive_pc_process()
    {
        uint8_t reg = pc_ -> readData();
        
        if(reg == 0) return;
        
        switch(reg)
        {
            case UsusamaProtocol::COMMAND_MOVE:
                move_command.enable = pc_->_register[UsusamaProtocol::COMMAND_MOVE];
                if(move_command.enable) robot_state_reply = UsusamaProtocol::RobotState::Move;
                if(!move_command.enable) robot_state_reply = UsusamaProtocol::RobotState::Stop;
                break;
                
            case UsusamaProtocol::COMMAND_POSE_X:
                move_command.x = UsusamaProtocol::decode_int2float( pc_->_register[UsusamaProtocol::COMMAND_POSE_X] );
                robot_state_reply = UsusamaProtocol::RobotState::Wait;
                break;
                
            case UsusamaProtocol::COMMAND_POSE_Y:
                move_command.y = UsusamaProtocol::decode_int2float( pc_->_register[UsusamaProtocol::COMMAND_POSE_Y] );
                robot_state_reply = UsusamaProtocol::RobotState::Wait;
                break;
            
            case UsusamaProtocol::COMMAND_POSE_THETA:
                move_command.theta = UsusamaProtocol::decode_int2float( pc_->_register[UsusamaProtocol::COMMAND_POSE_THETA] );
                robot_state_reply = UsusamaProtocol::RobotState::Wait;
                break;

            case UsusamaProtocol::COMMAND_RESET_ODOMETRY:
                robot_state_reply = UsusamaProtocol::RobotState::Reset;
                break;
        }
        last_command_ = reg;
    }
    
    private:
    UsusamaSerial *pc_;
    uint8_t last_command_ = 0;
    bool cmd_pose_changed_ = false;
    UsusamaProtocol::RobotState robot_state_reply;
    
    int n_reply_current_pose = 0;
    int n_reply_commad_pose = 0;
};

#endif