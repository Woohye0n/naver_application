#ifndef RR_STRATEGY_RR_STRATEGY_RR_STRATEGY_H_
#define RR_STRATEGY_RR_STRATEGY_RR_STRATEGY_H_

#include "rr_common/StateEstimated.h"
#include "rr_common/PerceptionObstacleArray.h"
#include "rr_common/MapWayPointXY.h"
#include "rr_common/SpeedControlTarget.h"
#include "rr_common/StrategyMode.h"
#include "rr_common/TimeToCollision.h"
#include "rr_common/PointXY.h"

#include <ros/ros.h>
#include <vector>
#include <memory>

namespace rr_strategy
{
    class Strategy
    {
    public:
        Strategy();
        ~Strategy();
        void Init(ros::NodeHandle &nh);
        void StateCb(const rr_common::StateEstimated::Ptr &state);
        void CollisionCb(const rr_common::TimeToCollision::Ptr &collision);
        void ObstacleCb(const rr_common::PerceptionObstacleArray::Ptr &obstacle);
        void SpinOnce();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif