#ifndef RR_STRATEGY_RR_STRATEGY_OBSTACLE_CHECKER_H_
#define RR_STRATEGY_RR_STRATEGY_OBSTACLE_CHECKER_H_

#include "rr_strategy/rr_strategy.h"
#include "rr_common/PerceptionObstacleArray.h"

#include <ros/ros.h>
#include <vector>
#include <memory>

namespace rr_strategy
{
    struct Obstacle
    {
        float x;
        float y;
        int idx;
    };

    class ObstacleChecker
    {
    public:
        ObstacleChecker();
        ~ObstacleChecker();
        void Init(ros::NodeHandle &nh);
        void SetObstacle(const rr_common::PerceptionObstacleArray::Ptr &obstacles);
        void SetMap(const std::vector<rr_common::PointXY> &map_in,
                    const std::vector<rr_common::PointXY> &map_out);
        void SetState(const rr_common::PointXY &state);
        void SetCurrentCourse(int current_course);
        void SetHeading(const float &heading);
        bool IsOk();
        int GetCourse();
        void SelectObstacle();
        void MakeStrategy();
        float CalDistancePath(rr_common::PointXY point1, rr_common::PointXY point2);
        void IsChanged();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // RR_STRATEGY_RR_STRATEGY_OBSTACLE_CHECKER_H_