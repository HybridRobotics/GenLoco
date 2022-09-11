#ifndef __POLICY_SIM_UDP_H__
#define __POLICY_SIM_UDP_H__

#include"udp_client_server.h"
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "std_msgs/Float32MultiArray.h"
#include "quadruped_config.h"
#include <math.h>
#include<iostream>
using namespace std;

#define BUFFER_MAX 100
namespace reinforce_udp{

    class Policy2sim
    {
    public:
        Policy2sim(quadruped_config);
        void udp_receive_Callback(const ros::TimerEvent& time_obj); 
        void cmd_Callback(const ros::TimerEvent&);
        void operator<<(quadruped_config);  // sim_config input using << 


        ros::Timer cmd_sending_timer;
        ros::Timer udp_recv_timer;
        float publish_frequency;
    private:
        void unpack_message();           // decode received information.
        void send_cmd();
        bool is_cmd_valid();            
        string robot_name;

        float udp_receive_frequency;
        float udp_time_out;
        float joint_upper_bound[3];
        float joint_lower_bound[3];
        string IP;
        int port;
        char data_buffer[BUFFER_MAX];
        double cmd_buffer[12];
        float init_pos[12];
        ros::NodeHandle n;
        udp_client_server::udp_server UDP;
        ros::Publisher cmd_pub;
        std_msgs::Float32MultiArray cmd_array;
    };

    class Sim2policy
    {
    public:
        Sim2policy(quadruped_config);
        void operator<<(quadruped_config);
    private:
        void subscriber_init(); //init subscriber
        void observation_callback(const std_msgs::Float32MultiArray&);
        void timerCallback(const ros::TimerEvent& time_obj); // send obs here

        string robot_name;
        float obs_send_frequency;
        string IP;
        int port;

        ros::NodeHandle n;
        udp_client_server::udp_client UDP;
        ros::Subscriber obs_sub;
        ros::Timer udp_send_timer;
        std_msgs::Float32MultiArray low_state_array;
    };

    
}

#endif