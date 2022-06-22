#include "rr_vms/gps_status_checker.h"

namespace rr_vms
{

    struct GPSStatusChecker::Impl
    {
        bool is_ok;
    };
    GPSStatusChecker::GPSStatusChecker() : impl_(new Impl) {}
    GPSStatusChecker::~GPSStatusChecker() {}
    void GPSStatusChecker::Init(ros::NodeHandle &nh)
    {
        impl_->is_ok = false;
    }

    bool GPSStatusChecker::IsOk()
    {
        return impl_->is_ok;
    }
    void GPSStatusChecker::CheckMessage(const ublox_msgs::NavPVT::Ptr &msg)
    {
        if (msg->flags == 131)
        {
            impl_->is_ok = true;
        }
        else
        {
            impl_->is_ok = false;
        }
    }
} // namespace rr_vms