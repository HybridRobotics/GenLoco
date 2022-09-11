#ifndef __MODE_DETECT_H__
#define __MODE_DETECT_H__

#include "ros/ros.h"
#include<iostream>
#include "std_msgs/UInt16.h"
#include <stdio.h>
#include <stdlib.h>
#include "quadruped_config.h"
enum {preload,stand_wait,stand_on,walk_on,damp_on};
class Mode_detect
{
public:
    Mode_detect(quadruped_config);
    void change_mode(int mode);
    int get_mode();
    ros::Publisher mode_pub;
    std_msgs::UInt16 mode;
private:
    ros::Timer send_timer;
    void timer_callback(const ros::TimerEvent&);
    std::string robot_name;
    float mode_frequency;
    ros::NodeHandle n;
};


#endif