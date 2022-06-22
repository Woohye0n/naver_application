#include "rr_safety_manager/rr_safety_manager.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rr_vms");
    ros::NodeHandle nh("~");
    rr_safety_manager::SafetyManager sm;
    sm.Init(nh);
    // select 2 * erp42 serial communication rate. never late.
    ros::Rate rate(40);
    while (ros::ok())
    {
        ros::spinOnce();
        sm.SpinOnce();
        rate.sleep();
    }
}