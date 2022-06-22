#include "rr_tracking_control/pure_persuit.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rr_pure_persuit");
    ros::NodeHandle nh("~");
    rr_control::PurePersuitController pp;
    pp.Init(nh);
    ros::spin();
}
