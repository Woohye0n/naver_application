#ifndef RR_VMS_RR_VMS_GPS_STATUS_CHECKER_H_
#define RR_VMS_RR_VMS_GPS_STATUS_CHECKER_H_

#include <ublox_msgs/NavPVT.h>
#include <ros/ros.h>
#include <memory>

namespace rr_vms
{
    class GPSStatusChecker
    {
    public:
        GPSStatusChecker();
        ~GPSStatusChecker();
        void Init(ros::NodeHandle &nh);
        bool IsOk();
        void CheckMessage(const ublox_msgs::NavPVT::Ptr &msg);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
} // namespace rr_vms

#endif //RR_VMS_RR_VMS_GPS_STATUS_CHECKER_H_
