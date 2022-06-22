#include "rr_fake_cam/fake_cam.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "fake_cam");
    ros::NodeHandle nh("~");
    rr_simul::MoraiCamConverter mcc;
    mcc.Init(nh);
    ros::Rate rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        mcc.SpinOnce();
        rate.sleep();
    }
}