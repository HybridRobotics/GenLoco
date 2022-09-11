#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include"std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt16.h"
#include <string>
#include <math.h>
#include "quadruped_config.h"
#include "mode_detect.h"
class stand_controller
{
public:
    stand_controller(quadruped_config);
    void stand(float secs);
    int get_current_mode();
    bool init_done();
    void sendDampCmd();
    void sendServoCmd();
    void set_stand_done();
private:

    bool low_state_received,mode_received;
    ros::Subscriber low_state_sub,mode_sub;
    ros::Timer timer;
    ros::Publisher stand_cmd_pub;
    ros::NodeHandle n;
    std::string robot_name;
    void low_state_callback(std_msgs::Float32MultiArray);
    void mode_callback(std_msgs::UInt16);
    void operator<<(quadruped_config);
    float Kp[3],Kd[3],init_pos[12];
    int current_mode;
    int stand_done;
    float low_state_array[19]={0},stand_cmd[13]={0};
    void send_timer_call_back(const ros::TimerEvent& event);
    void sendStandCmd();

};