#include "rr_object_detection/object_detector.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "obstacle_filter");
    ros::NodeHandle nh("~");
    rr_perception::ObjectDetector filter(nh);
    filter.Run();
}