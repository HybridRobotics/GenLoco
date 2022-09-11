#include "mode_detect.h"

Mode_detect::Mode_detect(quadruped_config config)
{
    robot_name = config.robot_name;
    mode.data = preload;
    mode_pub = n.advertise<std_msgs::UInt16>("/"+robot_name+"_cmd_mode",1);
    mode_frequency = 10.0;
    send_timer = n.createTimer(ros::Duration(1.0/mode_frequency),&Mode_detect::timer_callback,this);
}

int Mode_detect::get_mode()
{
    return mode.data;
}

void Mode_detect::change_mode(int mode_)
{
    mode.data = mode_;
}
void Mode_detect::timer_callback(const ros::TimerEvent& env)
{
    mode_pub.publish(mode);
}
