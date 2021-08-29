#include "mbed.h"
#include "platform/mbed_thread.h"
#include "i2cmaster.h"
#include "mecanum_interface.hpp"
#include "imu_interface.hpp"
#include "mecanum_controller.hpp"
#include "waypoint_generator.hpp"
#include "ususama_serial.h"
#include "ususama_controller.hpp"
#include <vector>
#include <list>


i2c master(p28, p27);
MecanumI2C mecanum(&master);
PMSUInterface pmsu(p9, p10);
MecanumController ususama(&mecanum, &pmsu);

int32_t Register[0x60] = {};
UsusamaSerial pc(USBTX, USBRX, Register, 115200);
UsusamaController commander(&pc);
BusOut leds(LED1, LED2, LED3, LED4);

Thread thread_read2pc;
Thread thread_write2pc;

void mecanum_test()
{
    ususama.ControllVelocity(Vel2D(1.0, 0.0, 0.0));
    thread_sleep_for(1000);
    ususama.ControllVelocity(Vel2D(0.0, 0.0, 0.0));
}

void imu_test()
{
    while(1)
    {
        printf("%d\r\n", (int)(pmsu.GetYawRadians() * RAD_TO_DEG));
    }
}

void read_pc_thread()
{
    while (true) {
        commander.receive_pc_process();
        ThisThread::sleep_for(103);
    }
}

void write_pc_thread()
{
    while (true) {
        commander.write_robot_state();
        ThisThread::sleep_for(50);
    }
}
int main()
{
    //std::list<std::unique_ptr<Pose2D>> ref_pose_list;
    //ref_pose_list.push_back(std::make_unique<Pose2D>(0.0, 0.8, 0.0));
    //ref_pose_list.push_back(std::make_unique<Pose2D>(0.0, 0.0, 0.0));
    //ref_pose_list.push_back(std::make_unique<Pose2D>(0.0, 0.5, 0));
    //ref_pose_list.push_back(std::make_unique<Pose2D>(0.0, 0.0, -1.57));
    //auto ref_pose_itr = ref_pose_list.begin();
    //control_test();
    //imu_test();
    
    ususama.ControllVelocity(Vel2D(0.0, 0.0, 0.0));
    // theta...
    WaypointGenerator<Pose2D> way_generator( Pose2D(0, 0, 0), Pose2D(0, 0, 0), 10.0);
    Timer t_way;
    t_way.start();
    
    thread_read2pc.start(read_pc_thread);
    thread_write2pc.start(write_pc_thread);

    bool is_way_generated = false;
    
    while(1)
    {
        //thread_sleep_for(10);
        
        ususama.ComputePose();
        commander.update(ususama.GetPose().x, ususama.GetPose().y, ususama.GetPose().theta);
        
        UsusamaProtocol::MoveCommand_t move_command = commander.getMoveCommand();
        Pose2D ref_pose(move_command.x, move_command.y, move_command.theta);

        if(commander.is_reset_called())
        {
            ususama.ResetPose();
            commander.notify_reset_odometry();
        }
        
        if(!move_command.enable)
        {
            ususama.PauseControl();
            is_way_generated = false;
            continue;
        }
        
        // Moveが許可されたときにwaypoint生成する
        if( commander.is_ref_pose_changed() || !is_way_generated)
        {
            // 0.5m/sとする
            // +0.8rad/s 
            double time_required = (abs(ref_pose.x-ususama.GetPose().x) + abs(ref_pose.y-ususama.GetPose().y))/0.5;
            time_required += abs(ref_pose.theta-ususama.GetPose().theta)/0.8;
            way_generator = WaypointGenerator<Pose2D>(
                ususama.GetPose(), 
                ref_pose, 
                time_required
            );
            is_way_generated = true;
            commander.notify_use_ref_pose();
            t_way.reset();
            t_way.start();
        }
        // waypoint存在していれば移動
        else
        {
            float t = t_way.read();
            ususama.ControlPosition( way_generator.GetPose( t ) );
        }

        if( ususama.IsReferencePose( ref_pose ) )
        {
            is_way_generated = false;
            ususama.PauseControl();
            commander.notify_reached_goal();
        }
        // 微調整長すぎるのでタイムアウトでゴールとする
        // over duration_time + 10 sec
        else if(t_way.read() > (way_generator.get_duration_time()+5))
        {
            is_way_generated = false;
            ususama.PauseControl();
            commander.notify_reached_goal();
        }
    }
}