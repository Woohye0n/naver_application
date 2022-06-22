#ifndef RR_VMS_RR_VMS_HEARTBEAT_CHECKER_H_
#define RR_VMS_RR_VMS_HEARTBEAT_CHECKER_H_

#include <ros/ros.h>
#include <memory>

namespace rr_vms
{
    class HearbeatChecker
    {
    public:
        HearbeatChecker();
        ~HearbeatChecker();
        void Init(ros::NodeHandle &nh,
                  const std::string &param_interval,
                  const std::string &param_jitter_allowed,
                  const std::string &device_name);
        bool IsOk();
        void SetStamp(const ros::Time &stamp);
        float GetAge();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

}
#endif // RR_VMS_RR_VMS_HEARTBEAT_CHECKER_H_