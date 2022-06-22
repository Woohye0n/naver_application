#ifndef RR_SIMUL_RR_LIDAR_MSG_CONVERTER_LIDAR_MSG_CONVERTER_H_
#define RR_SIMUL_RR_LIDAR_MSG_CONVERTER_LIDAR_MSG_CONVERTER_H_

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <vector>

namespace rr_simul
{
    class MoraiLidarConverter
    {
    public:
        MoraiLidarConverter();
        ~MoraiLidarConverter();

        void Init(ros::NodeHandle &nh);
        void VelodyneCallback(const sensor_msgs::PointCloud2::Ptr &point);
        void SpinOnce();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // RR_SIMUL_RR_LIDAR_MSG_CONVERTER_LIDAR_MSG_CONVERTER_H_
