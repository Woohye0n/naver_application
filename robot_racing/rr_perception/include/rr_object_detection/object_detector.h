#ifndef RR_PERCEPTION_RR_OBJECT_DETECTION_OBJECT_DETECTOR_H_
#define RR_PERCEPTION_RR_OBJECT_DETECTION_OBJECT_DETECTOR_H_

#include "rr_common/ERP42FeedbackExt.h"
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <memory>

namespace rr_perception
{

    class ObjectDetector
    {
    public:
        ObjectDetector(ros::NodeHandle &nh);
        virtual ~ObjectDetector();
        void Callback(const sensor_msgs::PointCloud2ConstPtr &input);
        void FeedbackCallback(const rr_common::ERP42FeedbackExt::Ptr &msg);
        void Run();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif //RR_PERCEPTION_RR_OBJECT_DETECTION_OBJECT_DETECTOR_H_