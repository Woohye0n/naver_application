#include "rr_vms/rr_vms.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rr_vms");
    ros::NodeHandle nh("~");
    rr_vms::VMS vms;
    vms.Init(nh);
    // select 2 * erp42 serial communication rate. never late.
    ros::Rate rate(40);
    while (ros::ok())
    {
        ros::spinOnce();
        vms.SpinOnce();
        rate.sleep();
    }
}