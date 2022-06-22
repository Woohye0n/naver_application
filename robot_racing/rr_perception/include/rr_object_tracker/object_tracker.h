#ifndef RR_PERCEPTION_RR_OBJECT_TRACKER_OBJECT_TRACKER_H_
#define RR_PERCEPTION_RR_OBJECT_TRACKER_OBJECT_TRACKER_H_

#include <memory>
#include <ros/ros.h>
#include "rr_common/PerceptionObstacleArray.h"

namespace rr_perception
{
    class ObjectTracker
    {
    public:
        ObjectTracker();
        ~ObjectTracker();
        void Init(ros::NodeHandle &nh);
        void Callback(const rr_common::PerceptionObstacleArray::Ptr &input);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
} // namespace rr_perception

#endif // RR_PERCEPTION_RR_OBJECT_TRACKER_OBJECT_TRACKER_H_
