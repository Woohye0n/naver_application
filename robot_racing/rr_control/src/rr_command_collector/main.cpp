#include "rr_command_collector/rr_command_collector.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rr_command_collector");
    ros::NodeHandle nh("~");
    rr_control::ControlCommandCollector collector;
    collector.Init(nh);
    ros::spin();
}