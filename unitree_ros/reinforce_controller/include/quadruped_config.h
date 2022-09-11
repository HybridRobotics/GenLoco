#ifndef __QUADRUPED_H__
#define __QUADRUPED_H__
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
#include <string.h>
#include <vector>
using namespace std;
class quadruped_config   // a class to save and share configuration.
{
public:
        quadruped_config();
        string robot_name;
        double control_frequency;
        double obs_send_frequency;
        vector<double> Kp;
        vector<double> Kd;
        double udp_receive_frequency;
        string recv_IP;     //receive cmd infor from which
        string send_IP;    //send obs infor to which
        int recv_port;             
        int send_port;
        vector<double> init_pos;
        vector<double> joint_upper_bound;
        vector<double> joint_lower_bound;
};

#endif
