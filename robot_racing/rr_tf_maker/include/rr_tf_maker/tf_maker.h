#ifndef RR_TF_MAKER_RR_TF_MAKER_TF_MAKER_H_
#define RR_TF_MAKER_RR_TF_MAKER_TF_MAKER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <rr_common/StateEstimated.h>
#include <sensor_msgs/PointCloud2.h>

namespace rr_tf_maker
{
    class TFMaker
    {
    public:
        TFMaker();
        ~TFMaker();

        void Init(ros::NodeHandle &nh);
        void CarPositionCallback(const rr_common::StateEstimated::Ptr &car);
        void VelodyneCallback(const sensor_msgs::PointCloud2::Ptr &msg);
        void SpinOnce();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // RR_TF_MAKER_RR_TF_MAKER_TF_MAKER_H_
