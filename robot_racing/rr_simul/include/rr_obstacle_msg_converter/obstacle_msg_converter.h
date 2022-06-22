#ifndef RR_SIMUL_RR_OBSTACLE_MSG_CONVERTER_OBSTACLE_MSG_CONVERTER_H_
#define RR_SIMUL_RR_OBSTACLE_MSG_CONVERTER_OBSTACLE_MSG_CONVERTER_H_

#include <morai_msgs/EgoVehicleStatus.h>
#include <rr_common/PerceptionObstacle.h>
#include <rr_common/PerceptionObstacleArray.h>
#include <morai_msgs/ObjectStatusList.h>
#include <ros/ros.h>
#include <vector>

namespace rr_simul
{
    class MoraiObstacleConverter
    {
    public:
        MoraiObstacleConverter();
        ~MoraiObstacleConverter();

        void Init(ros::NodeHandle &nh);
        void MoraiEgoCallback(const morai_msgs::EgoVehicleStatus::Ptr &ego);
        void MoraiObsCallback(const morai_msgs::ObjectStatusList::Ptr &obs);
        void SpinOnce();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // RR_SIMUL_RR_OBSTACLE_MSG_CONVERTER_OBSTACLE_MSG_CONVERTER_H_
