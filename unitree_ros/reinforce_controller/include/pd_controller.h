#ifndef __PD_CONTROLLER_H__
#define __PD_CONTROLLER_H__
#include <ros/ros.h>
#include <string>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32MultiArray.h"
#include "mode_detect.h"
#include "quadruped_config.h"
#include <stdio.h>
#include <stdlib.h>
namespace reinforce_controller
{

    class PD_controller
    {
    public:
        PD_controller(quadruped_config);
        int get_mode();
        unitree_legged_msgs::LowCmd load_cmd_by_mode();
        void update_low_state(unitree_legged_msgs::LowState ls);

    private:
        unitree_legged_msgs::LowCmd low_cmd,walk_cmd,stand_cmd,preload_cmd,damp_cmd;
        float walk_cmd_array[13],stand_cmd_array[13];
        std_msgs::Float32MultiArray low_state_array,logger_array;
        std::string robot_name;
        float obs_sending_freq;
        float Kp[3],Kd[3];
        int current_mode;
        bool imu_tune_start,yaw_initialized_done;
        bool stand_damp_mode,walk_damp_mode;
        double init_yaw,q_rot_yaw[4];
        ros::Subscriber walking_sub,standing_sub,mode_sub;
        ros::Publisher obs_sender,logger_sender;
        ros::Timer obs_send_Timer;
        ros::NodeHandle n;

        void compress_obs(const unitree_legged_msgs::LowState&);
        void record_data(const unitree_legged_msgs::LowState&);
        void cmd_init();
        void walking_sub_callback(const std_msgs::Float32MultiArray);
        void standing_sub_callback(const std_msgs::Float32MultiArray);
        void mode_sub_callback(const std_msgs::UInt16 _mode);
        void timer_call_back(const ros::TimerEvent& time_obj);
        double get_yaw_from_q(double,double,double,double);
    };

}

#endif