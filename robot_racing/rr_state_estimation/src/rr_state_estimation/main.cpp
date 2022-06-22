#include "rr_state_estimation/rr_state_estimation.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "state_estimation");
    ros::NodeHandle nh("~");
    rr_state_estimation::StateEstimation SE;
    SE.Init(nh);
    ros::spin();
}