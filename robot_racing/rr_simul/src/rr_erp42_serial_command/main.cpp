#include "rr_erp42_serial_command/erp42_serial_command.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "erp42_serial_command");
    ros::NodeHandle nh("~");
    rr_simul::MoraiSerialConverter msc;
    msc.Init(nh);
    ros::Rate rate(20);
    while (ros::ok())
    {
        ros::spinOnce();
        msc.SpinOnce();
        rate.sleep();
    }
}