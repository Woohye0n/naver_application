#ifndef RR_SAFETY_MANAGER_RR_SAFETY_MANAGER_RR_COURSE_OUT_DETECTOR_H_
#define RR_SAFETY_MANAGER_RR_SAFETY_MANAGER_RR_COURSE_OUT_DETECTOR_H_

#include "rr_common/MapWayPointXY.h"
#include "rr_common/StateEstimated.h"
#include "rr_common/PointXY.h"

#include <memory>
#include <ros/ros.h>
#include <vector>

namespace rr_safety_manager
{

    class CourseOutDetector
    {
    public:
        CourseOutDetector();
        ~CourseOutDetector();
        void Init(ros::NodeHandle &nh);

        bool IsOut();

        void StateCb(const rr_common::StateEstimated::Ptr &state_sub);
        void GetMapXY(ros::NodeHandle &nh);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;

        rr_common::PointXY FindClosestPoint(const std::vector<rr_common::PointXY> &way_points, rr_common::PointXY p);
    };
    float CalDistance(rr_common::PointXY point1, rr_common::PointXY point2);
};

#endif // RR_SAFETY_MANAGER_RR_SAFETY_MANAGER_RR_COURSE_OUT_DETECTOR_H_
