#include "rr_data_resource_manager/rr_data_resource_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_resource_manager");
    ros::NodeHandle nh("~");
    rr_data_resource_manager::DataResourceManager DRM;
    DRM.Init(nh);
    ros::spin();
}
