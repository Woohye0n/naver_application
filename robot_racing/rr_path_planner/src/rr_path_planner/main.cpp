#include "rr_path_planner/rr_path_planner.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rr_path_planner");
    ros::NodeHandle nh("~");
    rr_path_planner::LocalPathPlanner pp;
    pp.Init(nh);

    ros::Rate rate(20);
    while (ros::ok())
    {
        ros::spinOnce();
        pp.SpinOnce();
        rate.sleep();
    }
}