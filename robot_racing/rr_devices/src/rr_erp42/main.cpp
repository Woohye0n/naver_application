#include "rr_erp42/erp42.h"

int main(int argc, char *argv[])
{
    // init ros and get node handle with private namespace
    ros::init(argc, argv, "rr_erp42");
    ros::NodeHandle nh("~");

    rr_devices::ERP42 erp42;
    erp42.Initialize(nh);
    ros::Rate rate(20);
    while (ros::ok())
    {
        ros::spinOnce();
        erp42.SpinOnce();
        rate.sleep();
    }
}