#include "rr_obstacle_msg_converter/obstacle_msg_converter.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "obstacle_msg_converter");
    ros::NodeHandle nh("~");
    rr_simul::MoraiObstacleConverter moc;
    moc.Init(nh);
    ros::Rate rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        moc.SpinOnce();
        rate.sleep();
    }
}