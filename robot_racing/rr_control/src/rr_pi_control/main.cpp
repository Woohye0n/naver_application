#include "rr_pi_control/pi_control.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pi_controller");
    ros::NodeHandle nh("~");
    rr_control::PIController pi_ctrl;
    pi_ctrl.Init(nh);
    ros::spin();
}