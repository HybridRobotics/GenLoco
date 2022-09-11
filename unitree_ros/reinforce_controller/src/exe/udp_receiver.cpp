#include "policy_sim_udp.h"

int main(int argc,char **argv)
{
    ros::init(argc,argv,"UDP_Receiver");
    quadruped_config a1_gazebo_config;
    reinforce_udp::Policy2sim policy2sim(a1_gazebo_config);    
    // the udp receiver starts after construct this class

    ros::AsyncSpinner spinner(1); // use how many thread (0 represent all the threads)
    spinner.start(); //start callbacks
    cout<<endl<<"UDP receiver initialized!"<<endl;
    cout<<"Walking mode is allowed"<<endl;
    ros::waitForShutdown();
    ROS_INFO("UDP_Receiver Exit\n");
}
