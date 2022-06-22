#include "rr_lidar_msg_converter/lidar_msg_converter.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lidar_msg_converter");
    ros::NodeHandle nh("~");
    rr_simul::MoraiLidarConverter mlc;
    mlc.Init(nh);
    ros::Rate rate(20);
    while (ros::ok())
    {
        ros::spinOnce();
        mlc.SpinOnce();
        rate.sleep();
    }
}