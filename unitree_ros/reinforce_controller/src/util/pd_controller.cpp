#include "pd_controller.h"
using namespace reinforce_controller;

PD_controller::PD_controller(quadruped_config config)
{
    // import config
    robot_name = config.robot_name;
    obs_sending_freq = config.obs_send_frequency;
    for(int i=0;i<3;i++)       //to be improved
    {
        this->Kp[i] = config.Kp[i];
        this->Kd[i] = config.Kd[i];
    }

    // set status indicators.
    current_mode = preload;
    imu_tune_start = false;
    yaw_initialized_done = false;
    stand_damp_mode = true;
    walk_damp_mode = true;
    for(int i=0;i<20;i++)
        low_state_array.data.push_back(0.0);
    for(int i=0;i<24;i++)
        logger_array.data.push_back(0.0);
    // init cmd objs.
    cmd_init();

    // init subscribers and publishers
    walking_sub = n.subscribe("/" + robot_name + "_walking_cmd",1,&PD_controller::walking_sub_callback,this);
    standing_sub = n.subscribe("/"+robot_name + "_standing_cmd",1,&PD_controller::standing_sub_callback,this);
    mode_sub = n.subscribe("/"+robot_name+"_cmd_mode",1,&PD_controller::mode_sub_callback,this);
    obs_sender = n.advertise<std_msgs::Float32MultiArray>("/"+robot_name+"_lowstate",1);
    logger_sender = n.advertise<std_msgs::Float32MultiArray>("/"+robot_name+"_logger_data",1);
    // pay attention to 1.0. 1/1000 will end up as 0.
    obs_send_Timer = n.createTimer(ros::Duration(1.0/obs_sending_freq),&PD_controller::timer_call_back,this);


}
void PD_controller::cmd_init()
{
    preload_cmd.levelFlag = 0xff;      // init preload_cmd
    for(int i = 0; i<12; i++){
        preload_cmd.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        preload_cmd.motorCmd[i].q = (2.146E+9f);        // 禁止位置环
        preload_cmd.motorCmd[i].Kp = 0;
        preload_cmd.motorCmd[i].dq = (16000.0f);        // 禁止速度环
        preload_cmd.motorCmd[i].Kd = 0;
        preload_cmd.motorCmd[i].tau = 0;
    }

    for(int id=0;id<12;id++)           // init damp_cmd
    {
        damp_cmd.motorCmd[id].mode = 0x00; // Electronic braking mode.
    }

    //init walking_cmd and standing_cmd
    walk_cmd.levelFlag = 0xff; //set to lowlevel mode 
    for(int i=0; i<4; i++){ // 3 group , 4 joint per group.  // TODO
        walk_cmd.motorCmd[i*3+0].mode = 0x0A;
        walk_cmd.motorCmd[i*3+0].Kp = Kp[0];
        walk_cmd.motorCmd[i*3+0].dq = 0;
        walk_cmd.motorCmd[i*3+0].Kd = Kd[0];
        walk_cmd.motorCmd[i*3+0].tau = 0;
        walk_cmd.motorCmd[i*3+1].mode = 0x0A;
        walk_cmd.motorCmd[i*3+1].Kp = Kp[1];
        walk_cmd.motorCmd[i*3+1].dq = 0;
        walk_cmd.motorCmd[i*3+1].Kd = Kd[1];
        walk_cmd.motorCmd[i*3+1].tau = 0;
        walk_cmd.motorCmd[i*3+2].mode = 0x0A;
        walk_cmd.motorCmd[i*3+2].Kp = Kp[2];
        walk_cmd.motorCmd[i*3+2].dq = 0;
        walk_cmd.motorCmd[i*3+2].Kd = Kd[2];
        walk_cmd.motorCmd[i*3+2].tau = 0;
    }
    stand_cmd = walk_cmd;

}

int PD_controller::get_mode()
{
    return current_mode;
}
unitree_legged_msgs::LowCmd PD_controller::load_cmd_by_mode()
{
    if(current_mode == preload) 
    {
        low_cmd = preload_cmd;
    }
    else if(current_mode == stand_wait || current_mode ==stand_on)
    {
        if(!stand_damp_mode)
        {
            for(int i=0;i<12;i++)
                stand_cmd.motorCmd[i].q = stand_cmd_array[i];
            low_cmd = stand_cmd; 
        }
        else
        {
            low_cmd = damp_cmd;
        }

    }
    else if(current_mode == walk_on)
    {
        if(!walk_damp_mode)
        {
            for(int i=0;i<12;i++)
                walk_cmd.motorCmd[i].q = walk_cmd_array[i];
            low_cmd = walk_cmd;
        }
        else
        {
            low_cmd = damp_cmd;
        }

    }
    else if(current_mode == damp_on)
    {
        low_cmd = damp_cmd;
    }
    return low_cmd;
}

void PD_controller::update_low_state(unitree_legged_msgs::LowState ls)
{   
    unitree_legged_msgs::LowState obs_raw = ls;
    if(imu_tune_start==false)
        {}
    else if(yaw_initialized_done==false)                // calculate init yaw when standing is done.
    {
        
        double w=ls.imu.quaternion[0];
        double x = ls.imu.quaternion[1];
        double y = ls.imu.quaternion[2];
        double z = ls.imu.quaternion[3];
        init_yaw = get_yaw_from_q(w,x,y,z);
        std::cout<<std::endl<<"init_yaw after standing: "<<init_yaw/3.1415*180<<" degree"<<std::endl;
        std::cout<<"Compensated!"<<std::endl;

        // q_rot for rotating -yaw. 
        q_rot_yaw[0] = cos(-init_yaw/2.0);
        q_rot_yaw[1] =0;
        q_rot_yaw[2] =0;
        q_rot_yaw[3] = sin(-init_yaw/2.0);
        yaw_initialized_done = true;
    }
    else if(imu_tune_start&&yaw_initialized_done)        // remove init yaw from lowstate.
    {
        // q multiply.
        double w0 = ls.imu.quaternion[0];
        double x0 = ls.imu.quaternion[1];
        double y0 = ls.imu.quaternion[2];
        double z0 = ls.imu.quaternion[3];
        double w1 = q_rot_yaw[0];
        double x1 = q_rot_yaw[1];
        double y1 = q_rot_yaw[2];
        double z1 = q_rot_yaw[3];
        obs_raw.imu.quaternion[0] = -x1*x0 - y1*y0 - z1*z0 + w1*w0;
        obs_raw.imu.quaternion[1] = x1*w0 + y1*z0 - z1*y0 + w1*x0;
        obs_raw.imu.quaternion[2] = -x1*z0 + y1*w0 + z1*x0 + w1*y0;
        obs_raw.imu.quaternion[3] = x1*y0 - y1*x0 + z1*w0 + w1*z0;
    }
    compress_obs(obs_raw);
    record_data(obs_raw);
}

void PD_controller::compress_obs(const unitree_legged_msgs::LowState& ls)
{
    for(int i=0;i<4;i++)
        low_state_array.data[i] = ls.imu.quaternion[i];
    for(int i=0;i<3;i++)
        low_state_array.data[4+i] = ls.imu.gyroscope[i];
    for(int i=0;i<12;i++)
        low_state_array.data[7+i] = ls.motorState[i].q;
    low_state_array.data[19] = (current_mode==walk_on?1.0:0.0);
}

void PD_controller::record_data(const unitree_legged_msgs::LowState& ls)
{
    for(int i=0;i<12;i++)
    {
        logger_array.data[2*i] = ls.motorState[i].tauEst;
        logger_array.data[2*i+1] = ls.motorState[i].dq;
    }
}

void PD_controller::walking_sub_callback(const std_msgs::Float32MultiArray cmd)
{  
    for(int i=0;i<13;i++)
        walk_cmd_array[i] = cmd.data[i];
    walk_damp_mode = false;
    if(fabs(walk_cmd_array[12]+1)<0.0001) // array[12] == -1: damp cmd from walking controller.
    {
        walk_damp_mode = true;
    }
}


void PD_controller::standing_sub_callback(const std_msgs::Float32MultiArray cmd)
{
    for(int i=0;i<13;i++)
        stand_cmd_array[i] = cmd.data[i];

    stand_damp_mode = false;
    if(fabs(stand_cmd_array[12]-1)<0.0001)   // array[12] == 1 : standing done.
    {
        imu_tune_start=true;
    }
    else if(fabs(stand_cmd_array[12])<0.0001)  // array[12] == 0 : standing not done.
    {
        imu_tune_start=false;
        yaw_initialized_done=false;
    }
    else if(fabs(stand_cmd_array[12]+1)<0.0001) // array[12] == -1: damp cmd from standing controller.
    {
        stand_damp_mode = true;
    }
}

void PD_controller::mode_sub_callback(const std_msgs::UInt16 _mode)
{
    current_mode = _mode.data;
}

void PD_controller::timer_call_back(const ros::TimerEvent& time_obj)
{
    obs_sender.publish(low_state_array);
    logger_sender.publish(logger_array);
}


double PD_controller::get_yaw_from_q(double w,double x,double y,double z)
{
    double sin_1 = 2.0 *(w*z + x*y);
    double cos_1 = 1.0-2.0*(y*y + z*z);
    return atan2(sin_1,cos_1);
}
