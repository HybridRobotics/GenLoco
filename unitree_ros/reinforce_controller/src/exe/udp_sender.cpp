
#include "policy_sim_udp.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "UDP_Sender");
    quadruped_config a1_gazebo_config;
    reinforce_udp::Sim2policy sim2policy(a1_gazebo_config);
    
    ros::AsyncSpinner spinner(1);   // use how many thread (0 represent all the threads)

    spinner.start();//start callbacks
    cout<<"UDP sender initialized!"<<endl;
    usleep(300000); // must wait 300ms, to get first state
    ros::waitForShutdown();
    ROS_INFO("UDP_Sender Exit\n");

    return 0;
}