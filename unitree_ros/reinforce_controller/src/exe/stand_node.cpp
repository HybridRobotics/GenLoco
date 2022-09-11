#include "stand.h"
#include <string.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "stand_node");
    quadruped_config a1_config;
    stand_controller stand_ctr(a1_config);
    ros::spin();
    return 0;
}
