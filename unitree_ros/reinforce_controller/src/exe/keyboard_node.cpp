#include "mode_detect.h"
using namespace std;

void Print_current_mode(int mode)
{
    cout<<"current_mode: ";
    switch (mode)
    {
    case preload:
        cout<<"preloaad mode";
        break;
    case stand_wait:
        cout<<"stand_wait mode";
        break;
    case stand_on:
        cout<<"stand_on mode";
        break;
    case walk_on:
        cout<<"walk_on mode";
        break;
    case damp_on:
        cout<<"damp_on mode";
        break;
    default:
        break;
    }
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"keyboard_node");
    quadruped_config config;
    Mode_detect mode_detect(config);
    char c;
    sleep(2);
    cout<<"Press d to damp."<<endl;
    cout<<"Press s to start standing."<<endl;
    cout<<"Press w to walk."<<endl;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    while(c = getchar())
    {
        getchar();  //deal with enter
        switch (c)
        {
        case 'w':
            if(mode_detect.get_mode()!= stand_on)
            {
                cout<<"can not start walking. No standing up now!";
                break;
            }
            mode_detect.change_mode(walk_on);
            break;
        case 's':
            if(mode_detect.get_mode()!= stand_wait)
            {
                mode_detect.change_mode(stand_wait);
                cout<<"Press s again to start standing up!"<<endl;
                break;
            }
            else if(mode_detect.get_mode()== stand_wait)
            {
                mode_detect.change_mode(stand_on);
                break;
            }

        case 'd':
            mode_detect.change_mode(damp_on);

            break;
        default:
            break;
        }
        Print_current_mode(mode_detect.get_mode());
    }
    ros::waitForShutdown();

}