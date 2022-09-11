#include "quadruped_config.h"

quadruped_config::quadruped_config()
{
    ros::NodeHandle n;
    n.getParam("/rl_control_config/robot_name",this->robot_name);
    n.getParam("/rl_control_config/control_frequency",this->control_frequency);
    n.getParam("/rl_control_config/obs_send_frequency",this->obs_send_frequency);
    n.getParam("/rl_control_config/Kp",this->Kp);
    n.getParam("/rl_control_config/Kd",this->Kd);
    n.getParam("/rl_control_config/udp_receive_frequency",this->udp_receive_frequency);
    n.getParam("/rl_control_config/recv_IP",this->recv_IP);
    n.getParam("/rl_control_config/send_IP",this->send_IP);
    n.getParam("/rl_control_config/recv_port",this->recv_port);
    n.getParam("/rl_control_config/send_port",this->send_port);
    n.getParam("/rl_control_config/init_pos",this->init_pos);
    n.getParam("/rl_control_config/joint_upper_bound",this->joint_upper_bound);
    n.getParam("/rl_control_config/joint_lower_bound",this->joint_lower_bound);
}
