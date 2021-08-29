#ifndef MECANUM_CONTROLLER_HPP
#define MECANUM_CONTROLLER_HPP

#include "mecanum_interface.hpp"
#include "imu_interface.hpp"
#include "pid_controller.hpp"
#include "math.h"

#include <ctime>
#include <vector>

//const double PI = 3.14159265359;
const float DEG_TO_RAD = PI/180.0;
const float RAD_TO_DEG = 180.0/PI;

using namespace std::chrono;

class Pose2D
{
    public:
    Pose2D(double x=0, double y=0, double theta=0) : x(x), y(y), theta(theta)
    {
    }
    Pose2D(double x, double y, double theta, double time) : x(x), y(y), theta(theta), time_stamp(time)
    {
    }
    Pose2D operator +(Pose2D ref)
    {
        return Pose2D(x + ref.x, y + ref.y, theta + ref.theta);
    }
    Pose2D operator -(Pose2D ref)
    {
        return Pose2D(x - ref.x, y - ref.y, theta - ref.theta);
    }
    Pose2D operator *(Pose2D ref)
    {
        return Pose2D(x * ref.x, y * ref.y, theta * ref.theta);
    }
    Pose2D operator *(double val)
    {
        return Pose2D(x * val, y * val, theta * val);
    }
    /*Pose2D operator /(Pose2D ref)
    {
        return Pose2D(x / ref.x, y / ref.y, theta / ref.theta);
    }*/
    double operator /(Pose2D ref)
    {
        return std::max( std::max( abs(x / ref.x), abs(y / ref.y) ), abs(theta / ref.theta) );
    }
    Pose2D operator /(double val)
    {
        return Pose2D(x / val, y / val, theta / val);
    }
    void ConstraintUpper(Pose2D ref, Pose2D dir)
    {
        if(dir.x > 0 && x > ref.x) x = ref.x;
        if(dir.y > 0 && y > ref.y) y = ref.y;
        if(dir.theta > 0 && theta > ref.theta) theta = ref.theta; 
        if(dir.x < 0 && x < ref.x) x = ref.x;
        if(dir.y < 0 && y < ref.y) y = ref.y;
        if(dir.theta < 0 && theta < ref.theta) theta = ref.theta;  
    }
    void ConstraintLower(Pose2D ref, Pose2D dir)
    {
        if(dir.x > 0 && x < ref.x) x = ref.x;
        if(dir.y > 0 && y < ref.y) y = ref.y;
        if(dir.theta > 0 && theta < ref.theta) theta = ref.theta; 
        if(dir.x < 0 && x > ref.x) x = ref.x;
        if(dir.y < 0 && y > ref.y) y = ref.y;
        if(dir.theta < 0 && theta > ref.theta) theta = ref.theta; 
    }
    double x = 0;
    double y = 0;
    double theta = 0;
    double time_stamp = 0;
};

class Vel2D
{
    public:
    Vel2D(double x, double y, double theta) : x(x), y(y), theta(theta)
    {
    }
    double x = 0;
    double y = 0;
    double theta = 0;
};

struct MecanumParameter
{
    const double L = 0.233; //233 [mm]
    const double wheel_radius = 0.0635;     // メカナム半径[m]
    const double deg_per_pulse = 0.0072;   // ステッピングモータ(AR46SAK-PS50-1)
};

class ElapsedTimer
{
    public:
    ElapsedTimer()
    {
        timer_.start();
        last_us_ = chrono::duration_cast<chrono::microseconds>(timer_.elapsed_time()).count();
    }
    
    double GetElapsedSeconds()
    {
        if(paused_)
        {
            timer_.start();
            paused_ = false;
        }
        uint64_t current_us = chrono::duration_cast<chrono::microseconds>(timer_.elapsed_time()).count();
        double dt = (double)(current_us - last_us_) / 1000000;
        last_us_ = current_us;
        return dt;
    }

    // timerを一時停止
    // GetElapsedSeconds()呼び出しで再開
    void pause()
    {
        timer_.stop();
        paused_ = true;
    }
    
    private:
    Timer timer_;
    uint64_t last_us_ = 0;
    bool paused_ = false;
};

class PoseController
{
    public:
    PoseController()
    {        
        /*id_name_.emplace_back("pose_x");
        id_name_.emplace_back("pose_y");
        id_name_.emplace_back("pose_theta");
        
        for(int i=0; i<id_name_.size(); i++) {
            id_map_[id_name_[i]] = i;
        }*/
        //pid_param_.resize(id_name_.size());
        //controller_.resize(id_name_.size(), 0);
        pid_param_.resize(3);
        controller_.resize(3);
        
        // for pose x
        pid_param_[0].bound.upper = 1.0;
        pid_param_[0].bound.lower = -1.0;
        pid_param_[0].threshold.upper = 0.001;
        pid_param_[0].threshold.lower = -0.001;
        pid_param_[0].kp = 1.5;
        pid_param_[0].ki = 0.06;
        pid_param_[0].kd = 0.0;
        
        // for pose y
        pid_param_[1].bound.upper = 1.0;
        pid_param_[1].bound.lower = -1.0;
        pid_param_[1].threshold.upper = 0.005;
        pid_param_[1].threshold.lower = -0.005;
        pid_param_[1].kp = 1.5;
        pid_param_[1].ki = 0.06;
        pid_param_[1].kd = 0.0;
        
        // for pose theta
        pid_param_[2].bound.upper = 1.0;
        pid_param_[2].bound.lower = -1.0;
        pid_param_[2].threshold.upper = 0.02;
        pid_param_[2].threshold.lower = -0.02;
        pid_param_[2].kp = 3.00;
        pid_param_[2].ki = 0.10;
        pid_param_[2].kd = 0.0;
        
        controller_[0] = std::make_unique<PIDControl>(pid_param_[0]);
        controller_[1] = std::make_unique<PIDControl>(pid_param_[1]);
        controller_[2] = std::make_unique<PIDControl>(pid_param_[2]);
        //controller_[0] -> parameter(pid_param_[0]);
        //controller_[1] -> parameter(pid_param_[1]);
        //controller_[2] -> parameter(pid_param_[2]);
    }
    
    protected:
    std::vector<PIDControl::Parameter> pid_param_;   //[x, y, theta]
    //std::map<std::string, uint16_t> id_map_;
    //std::vector<std::string> id_name_;
    std::vector<std::unique_ptr<PIDControl>> controller_;
};

class MecanumController : PoseController
{
    public:
    MecanumController(MecanumInterface *mecanum, IMUInterface *imu) : pose_(0, 0, 0), PoseController()
    {
        mecanum_ = mecanum;
        imu_ = imu;
    }
    
    void ControllVelocity(Vel2D vel)
    {
        double v[MOTOR_NUM] = {};
        // 各車輪の速度
        v[0] = -vel.x + vel.y - param_.L * vel.theta;  // m/sec
        v[1] = -vel.x - vel.y - param_.L * vel.theta;
        v[2] =  vel.x - vel.y - param_.L * vel.theta;
        v[3] =  vel.x + vel.y - param_.L * vel.theta;
        
        // 各モータのパルスに変換
        for(int i = 0; i < MOTOR_NUM; i++)
        {
            float f = v[i] / param_.wheel_radius * RAD_TO_DEG / param_.deg_per_pulse;
            wheel_frequency_[i] = mecanum_ -> ControlMotor(i, (int)f);
        }
    }
    
    // 周波数を保存する必要があるのでここに実装
    void CoastAllMotor()
    {
        for(int i = 0; i < MOTOR_NUM; i++)
        {
            // オドメトリ用に周波数を保存
            wheel_frequency_[i] = mecanum_ -> ControlMotor(i, 0);
        }
    }
    
    void ControlPosition(Pose2D ref_pose)
    {
        //Pose2D pose_err = ref_pose - pose_;
        // to do: IMUが反応する最小角速度で制限


        double dt = t_controller_.GetElapsedSeconds();
        // 慣性座標での速度を計算
        double xI_dot = controller_[0] -> calculate(ref_pose.x, pose_.x, dt);
        double yI_dot = controller_[1] -> calculate(ref_pose.y, pose_.y, dt);
        double thetaI_dot = controller_[2] -> calculate(ref_pose.theta, pose_.theta, dt);
        // ロボット座標での速度に変換
        double x_dot = cos(pose_.theta) * xI_dot + sin(pose_.theta) * yI_dot;
        double y_dot = -sin(pose_.theta) * xI_dot + cos(pose_.theta) * yI_dot;

        if(abs(thetaI_dot) < 0.0001)
        {
            thetaI_dot = 0;
        }
        else
        {
            if(thetaI_dot < 0)
                thetaI_dot = min(thetaI_dot, -imu_lower_limit);
            else if(thetaI_dot > 0)
                thetaI_dot = max(thetaI_dot, imu_lower_limit);
        }
        ControllVelocity( Vel2D(x_dot, y_dot,  thetaI_dot) );
    }

    void PauseControl()
    {
        t_controller_.pause();
        ControllVelocity(Vel2D(0.0, 0.0, 0.0));
    }
    
    bool IsReferencePose(Pose2D ref_pose)
    {
        Pose2D pose_err = ref_pose - pose_;
        bool is_refpose = (pose_err.x < pid_param_[0].threshold.upper) && (pose_err.x > pid_param_[0].threshold.lower);
            is_refpose &= (pose_err.y < pid_param_[1].threshold.upper) && (pose_err.y > pid_param_[1].threshold.lower);
            is_refpose &= (pose_err.theta < pid_param_[2].threshold.upper) && (pose_err.theta > pid_param_[2].threshold.lower);
        return is_refpose;
    }

    bool IsSpeedZero()
    {
        bool is_zero = true;
        is_zero &= (wheel_frequency_[0] < 50);
        is_zero &= (wheel_frequency_[1] < 50);
        is_zero &= (wheel_frequency_[2] < 50);
        is_zero &= (wheel_frequency_[3] < 50);
        return is_zero;
    }
    
    Pose2D GetPose()
    {
        return pose_;
    }
    
    void UpdateWheel()
    {
        double dt = t_.GetElapsedSeconds();
        for(int i = 0; i < MOTOR_NUM; i++)
        {
            wheel_rad_[i] = param_.deg_per_pulse * (double)wheel_frequency_[i] * dt * DEG_TO_RAD;
        }
    }
    
    void UpdateTheta()
    {
        pose_.theta = imu_ -> GetYawRadians() - theta_offset_;
    }
    
    void ComputePose()
    {
        UpdateWheel();
        UpdateTheta();
        double dx = (-wheel_rad_[0] - wheel_rad_[1] + wheel_rad_[2] + wheel_rad_[3]) / 4 * param_.wheel_radius;
        double dy = ( wheel_rad_[0] - wheel_rad_[1] - wheel_rad_[2] + wheel_rad_[3]) / 4 * param_.wheel_radius;
        pose_.x += dx * cos(pose_.theta) + dy * sin(-pose_.theta);
        pose_.y += dy * cos(pose_.theta) + dx * sin(pose_.theta);
    }

    void ResetPose()
    {
        pose_.x = 0;
        pose_.y = 0;
        theta_offset_ = imu_ -> GetYawRadians();
    }
    
    private:
    MecanumInterface *mecanum_;
    IMUInterface *imu_;
    MecanumParameter param_;
    double theta_offset_ = 0;
    // IMUが反応できる角速度の小さいほうの制限
    //0.006 rad/sからあぶない
    const double imu_lower_limit = 0.008;
    
    Pose2D pose_;
    double wheel_rad_[4];
    int64_t wheel_frequency_[4];
    ElapsedTimer t_;
    ElapsedTimer t_controller_;
};

#endif