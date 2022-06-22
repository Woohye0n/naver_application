#include "rr_tf_maker/tf_maker.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rr_tf_maker");
    ros::NodeHandle nh("~");
    rr_tf_maker::TFMaker tfm;
    tfm.Init(nh);
    ros::Rate rate(20);
    while (ros::ok())
    {
        ros::spinOnce();
        tfm.SpinOnce();
        rate.sleep();
    }
}
