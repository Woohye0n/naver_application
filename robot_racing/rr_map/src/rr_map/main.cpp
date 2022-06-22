#include "rr_map/rr_map.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map");
    ros::NodeHandle nh("~");
    rr_map::Map map;
    map.Init(nh);

    ros::spin();

    return 0;
}
