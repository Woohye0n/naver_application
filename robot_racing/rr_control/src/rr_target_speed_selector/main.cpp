#include "rr_target_speed_selector/rr_target_speed_selector.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rr_target_speed_selector");
    ros::NodeHandle nh("~");
    rr_control::TargetSpeedSelector speed_select;
    speed_select.Init(nh);
    ros::spin();
}