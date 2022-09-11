#include "stand.h"

void stand_controller::operator<<(quadruped_config config)
{
    this->robot_name = config.robot_name;

    for(int i=0;i<12;i++)       //to be improved
    {
        this->init_pos[i] = config.init_pos[i];
    }
}
stand_controller::stand_controller(quadruped_config config)
{
    (*this)<<config;
    low_state_sub = n.subscribe("/"+robot_name+"_lowstate",1,&stand_controller::low_state_callback,this);
    mode_sub = n.subscribe("/" + robot_name + "_cmd_mode",1,&stand_controller::mode_callback,this);
    stand_cmd_pub = n.advertise<std_msgs::Float32MultiArray>("/"+robot_name+"_standing_cmd",1);
    timer = n.createTimer(ros::Duration(1.0/1000.0),&stand_controller::send_timer_call_back,this);
    current_mode=preload;
    stand_done = 0;
}

void stand_controller::low_state_callback(std_msgs::Float32MultiArray ls)
{
    for(int i=0;i<19;i++)
        low_state_array[i] = ls.data[i];
}


void stand_controller::sendServoCmd()
{
    std_msgs::Float32MultiArray sending_msg;
    for(int i=0;i<12;i++)
        sending_msg.data.push_back(stand_cmd[i]);
    sending_msg.data.push_back(stand_done);
    stand_cmd_pub.publish(sending_msg);
}

void stand_controller::stand(float duration_secs)
{
    double pos[12] ,lastPos[12], percent;
    for(int j=0; j<12; j++) lastPos[j] = low_state_array[7+j];
    double duration = duration_secs * 1000.0;
    for(int i=1; i<=duration; i++){
        if(!ros::ok()) break;
        percent = (double)i/duration;
        for(int j=0; j<12; j++)
            stand_cmd[j] = lastPos[j]*(1-percent) + init_pos[j]*percent; 
        sendServoCmd();
        usleep(1000);
    }
}
void stand_controller::sendStandCmd()
{
    std_msgs::Float32MultiArray sending_msg;
    for(int i=0;i<12;i++)
        sending_msg.data.push_back(init_pos[i]);
    sending_msg.data.push_back(1);
    stand_cmd_pub.publish(sending_msg);
}
void stand_controller::mode_callback(std_msgs::UInt16 _mode)
{
    current_mode = _mode.data;
}


int stand_controller::get_current_mode()
{
    return current_mode;
}


bool stand_controller::init_done()
{
    return low_state_received&&mode_received;
}

void stand_controller::sendDampCmd()
{
    std_msgs::Float32MultiArray sending_msg;
    for(int i=0;i<12;i++)
        sending_msg.data.push_back(0);
    sending_msg.data.push_back(-1);
    stand_cmd_pub.publish(sending_msg);
}


void stand_controller::send_timer_call_back(const ros::TimerEvent& event)
{
    if (stand_done == 1 && current_mode !=damp_on)
        sendServoCmd();
    else if(current_mode==stand_on)
    {
        stand(2.0); // 2.0 secs to stand
        stand_done = 1;
    }
    else
    {
        sendDampCmd();
        stand_done = 0;
    }
}