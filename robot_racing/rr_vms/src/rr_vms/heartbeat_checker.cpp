#include "rr_vms/heartbeat_checker.h"

namespace rr_vms
{
    struct HearbeatChecker::Impl
    {
        float age;
        float interval;
        float jitter_allowed;
        std::string dev_name;
        ros::Time prev_stamp;
    };

    HearbeatChecker::HearbeatChecker() : impl_(new Impl) {}
    HearbeatChecker::~HearbeatChecker() {}

    void HearbeatChecker::Init(ros::NodeHandle &nh,
                               const std::string &param_interval,
                               const std::string &param_jitter_allowed,
                               const std::string &device_name)
    {
        ROS_ASSERT(nh.getParam(param_interval, impl_->interval));
        ROS_ASSERT(nh.getParam(param_interval, impl_->jitter_allowed));

        impl_->dev_name = device_name;
        impl_->age = 0.0;
        impl_->prev_stamp = ros::Time::now();
    }

    bool HearbeatChecker::IsOk()
    {
        impl_->age = (ros::Time::now() - impl_->prev_stamp).toSec();
        return impl_->age < (impl_->interval + impl_->jitter_allowed);
    }
    float HearbeatChecker::GetAge()
    {
        return impl_->age;
    }

    void HearbeatChecker::SetStamp(const ros::Time &stamp)
    {
        impl_->prev_stamp = stamp;
    }

} // namespace rr_vms