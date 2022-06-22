#ifndef RR_PATH_PLANNER_RR_PATH_PLANNER_RR_HEADING_CHECKER_H_
#define RR_PATH_PLANNER_RR_PATH_PLANNER_RR_HEADING_CHECKER_H_

#include "rr_common/PointXY.h"

#include <ros/ros.h>
#include <vector>
#include <memory>

namespace rr_path_planner
{
    class HeadingChecker
    {
    public:
        HeadingChecker();
        ~HeadingChecker();
        void Init(ros::NodeHandle &nh,
                  const std::string &deg_diff_allowed);
        bool IsOk();
        void SetPath(const std::vector<rr_common::PointXY> &path);
        void SetHeading(const float &heading);
        void SetState(rr_common::PointXY &state);
        float CalDistanceHead(rr_common::PointXY point1, rr_common::PointXY point2);
        float FindMinDistanceHead(const std::vector<rr_common::PointXY> &points, rr_common::PointXY src);
        void FindWL();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

}
#endif // RR_PATH_PLANNER_RR_PATH_PLANNER_RR_HEADING_CHECKER_H_