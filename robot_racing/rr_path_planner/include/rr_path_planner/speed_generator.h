#ifndef RR_PATH_PLANNER_RR_PATH_PLANNER_SPEED_GENERATOR_H_
#define RR_PATH_PLANNER_RR_PATH_PLANNER_SPEED_GENERATOR_H_

#include "rr_common/PointXY.h"

#include <Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include <memory>

namespace rr_path_planner
{
    class SpeedGenerator
    {
    public:
        SpeedGenerator();
        ~SpeedGenerator();
        void Init(ros::NodeHandle &nh);
        void SetPath(std::vector<rr_common::PointXY> &path);
        void SetState(const rr_common::PointXY &state);
        float GetSpeed();
        float CalCurvature(const Eigen::Vector2f &p0,
                           const Eigen::Vector2f &p1,
                           const Eigen::Vector2f &p2);
        void MakeVector();
        float CalDistanceSpeed(rr_common::PointXY point1, rr_common::PointXY point2);
        rr_common::PointXY FindClosestPointSpeed(const std::vector<rr_common::PointXY> &points, rr_common::PointXY src);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // RR_PATH_PLANNER_RR_PATH_PLANNER_SPEED_GENERATOR_H_