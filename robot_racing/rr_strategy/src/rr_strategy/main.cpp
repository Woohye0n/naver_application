#include "rr_strategy/rr_strategy.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "strategy");
    ros::NodeHandle nh("~");
    rr_strategy::Strategy strategy;
    strategy.Init(nh);
    ros::Rate rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        strategy.SpinOnce();
        rate.sleep();
    }
}