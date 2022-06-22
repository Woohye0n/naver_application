#include "rr_gps_msg_converter/gps_msg_converter.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gps_msg_converter");
    ros::NodeHandle nh("~");
    rr_simul::MoraiGpsConverter mgc;
    mgc.Init(nh);
    ros::Rate rate(20);
    while (ros::ok())
    {
        ros::spinOnce();
        mgc.SpinOnce();
        rate.sleep();
    }
}