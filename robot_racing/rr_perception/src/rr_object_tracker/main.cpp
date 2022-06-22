#include "rr_object_tracker/object_tracker.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "object_tracker");
    ros::NodeHandle nh("~");
    rr_perception::ObjectTracker trk;
    trk.Init(nh);
    ros::spin();
}