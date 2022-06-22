#ifndef RR_VMS_RR_VMS_DRIVE_MODE_CHECKER_H_
#define RR_VMS_RR_VMS_DRIVE_MODE_CHECKER_H_

#include "rr_common/ERP42FeedbackRaw.h"
#include <ros/ros.h>
#include <memory>

namespace rr_vms
{
    class DriveModeChecker
    {
    public:
        DriveModeChecker();
        ~DriveModeChecker();
        void Init(ros::NodeHandle &nh);
        bool IsOk();
        void CheckMessage(const rr_common::ERP42FeedbackRaw::Ptr &msg);
        std::string ModeString();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
} // namespace rr_vms

#endif //RR_VMS_RR_VMS_GPS_STATUS_CHECKER_H_
