#ifndef RR_PATH_PLANNER_RR_PATH_PLANNER_RR_PATH_PLANNER_H_
#define RR_PATH_PLANNER_RR_PATH_PLANNER_RR_PATH_PLANNER_H_

#include "rr_common/StateEstimated.h"
#include "rr_common/StrategyMode.h"
#include "rr_common/LocalPathPoints.h"
#include "rr_common/SpeedControlTarget.h"

#include <ros/ros.h>
#include <memory>

namespace rr_path_planner
{
    class LocalPathPlanner
    {
    public:
        LocalPathPlanner();
        ~LocalPathPlanner();
        void Init(ros::NodeHandle &nh);
        void StateCallback(const rr_common::StateEstimated::Ptr &state);
        void StrategyCallback(const rr_common::StrategyMode::Ptr &course);
        void SpinOnce();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // RR_PATH_PLANNER_RR_PATH_PLANNER_RR_PATH_PLANNER_H_