#ifndef RR_PATH_PLANNER_RR_PATH_PLANNER_ON_COURSE_CHECKER_H_
#define RR_PATH_PLANNER_RR_PATH_PLANNER_ON_COURSE_CHECKER_H_

#include "rr_common/PointXY.h"

#include <ros/ros.h>
#include <vector>
#include <memory>

namespace rr_path_planner
{
    class OnCourseChecker
    {
    public:
        OnCourseChecker();
        ~OnCourseChecker();
        void Init(ros::NodeHandle &nh,
                  const std::string &path_diff_allowed);
        bool IsOk();
        void SetPath(const std::vector<rr_common::PointXY> &path);
        void SetState(const rr_common::PointXY &state);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
        float CalDistanceOn(rr_common::PointXY point1, rr_common::PointXY point2);
        float FindMinDistanceOn(const std::vector<rr_common::PointXY> &points, rr_common::PointXY src);
    };

} // namespace rr_path_planner

#endif // RR_PATH_PLANNER_RR_PATH_PLANNER_ON_COURSE_CHECKER_H_