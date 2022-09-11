#include "policy_sim_udp.h"

using namespace std;
using namespace reinforce_udp;

Policy2sim::Policy2sim(quadruped_config config)
    :UDP(IP,port),IP(config.recv_IP),port(config.recv_port)
{
    (*this)<<config;
    for(int i=0; i<13; i++)
    {
        cmd_array.data.push_back(0.0);

    }
    cmd_pub = n.advertise<std_msgs::Float32MultiArray>("/"+robot_name +"_walking_cmd",1);
    cout<<"Waiting for udp signals"<<endl;
    UDP.recv(data_buffer,100); // waiting for udp signals 

    udp_recv_timer = n.createTimer(ros::Duration(1.0/udp_receive_frequency),&Policy2sim::udp_receive_Callback,this);
    cmd_sending_timer = n.createTimer(ros::Duration(1.0/publish_frequency),&Policy2sim::cmd_Callback,this);
}

void Policy2sim::operator<<(quadruped_config config)
{
    this->robot_name = config.robot_name;
    this->publish_frequency = config.control_frequency;
    this->udp_receive_frequency = config.udp_receive_frequency;
    for(int i=0;i<3;i++)       //to be improved
    {
        this->joint_upper_bound[i] = config.joint_upper_bound[i];
        this->joint_lower_bound[i] = config.joint_lower_bound[i];
    }
}

void Policy2sim::unpack_message()  
{
    vector<string> result;
    string sep=" ";
    string::size_type pos;
    string data=data_buffer;
    data += sep;
    int data_size = data.size();
    for(int i=0;i<data_size;i++)
    {
        pos = data.find(sep,i);
        if(pos<data_size)
        {
            string s=data.substr(i,pos-i);
            result.push_back(s);
            i = pos;
        }
    }
    if(result.size()!=12)
        // cout<<"broken cmd with "<<result.size()<<"nums"<<endl;
        {}
    for(int i=0; i<12; i++)
    {
        cmd_buffer[i]=atof(result[i].c_str());
        // cout<<cmd_buffer[i]<<endl;
    }
}

void Policy2sim::udp_receive_Callback(const ros::TimerEvent& time_obj)
{
    UDP.recv(data_buffer,100);
    // TODO deal with timeout

}

void Policy2sim::cmd_Callback(const ros::TimerEvent& timer)
{
    send_cmd();
}
void Policy2sim::send_cmd()
{
    this->unpack_message();
    for(int i=0;i<12;i++)
    {
        cmd_array.data[i] = cmd_buffer[i];
    }

    cmd_array.data[12] = (is_cmd_valid()?1:-1);
    cmd_pub.publish(cmd_array);
}

 bool Policy2sim::is_cmd_valid()
 {
     for(int i=0;i<12;i++)
     {
        if(cmd_array.data[i]-joint_lower_bound[i%3]<=-1 || cmd_array.data[i] - joint_upper_bound[i%3]>=1)
        {
            cout<<"Invalid cmd"<<i<<" "<<cmd_array.data[i]<<endl;
            // for(int j=0;j<12;j++)
            //     cout<<i<<" "<<cmd_array.data[j];
            // cout<<endl;
            return false;
        }
        else if (cmd_array.data[i] >= joint_lower_bound[i%3] && cmd_array.data[i] <= joint_upper_bound[i%3])
        {}
        else
        {
            cout<<"need clip"<<i<<" "<<cmd_array.data[i];
            cmd_array.data[i] = cmd_array.data[i] < joint_lower_bound[i%3]?joint_lower_bound[i%3]:joint_upper_bound[i%3];
            cout<<" after clip"<<cmd_array.data[i]<<endl;
        }
     }
     return true;
 }


Sim2policy::Sim2policy(quadruped_config config)
    :UDP(IP,port),IP(config.send_IP),port(config.send_port)
{
    (*this)<<config;
    subscriber_init();
    low_state_array.data.push_back(1.0);  // make sure the wxyz makes sense.
    for(int i=1;i<20;i++)
        low_state_array.data.push_back(0.0);
    
}

void Sim2policy::operator<<(quadruped_config config)
{
    this->robot_name = config.robot_name;
    this->obs_send_frequency = config.obs_send_frequency;
}
void Sim2policy::observation_callback(const std_msgs::Float32MultiArray& ls)
{
    for(int i=0;i<20;i++)
        this->low_state_array.data[i] = ls.data[i];
}

void Sim2policy::subscriber_init()
{

    obs_sub = n.subscribe("/"+robot_name+"_lowstate",1,&Sim2policy::observation_callback,this);
    udp_send_timer = n.createTimer(ros::Duration(1.0/obs_send_frequency),&Sim2policy::timerCallback,this);
}

void Sim2policy::timerCallback(const ros::TimerEvent& time_obj)
{
    char obs_raw_buffer[10];
    string observation("");
    string space(" ");
    //encode them into string.
    for(int i=0;i<20;i++)   
    {
        gcvt(low_state_array.data[i],3,obs_raw_buffer);
        string ob=obs_raw_buffer;
        if(i!=0)observation += space;
        observation += ob;
    }

    UDP.send(observation.c_str(),strlen(observation.c_str()));

}




