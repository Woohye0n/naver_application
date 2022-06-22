#ifndef RR_SAFETY_MANAGER_RR_SAFETY_MANAGER_RR_SAFETY_MANAGER_H_
#define RR_SAFETY_MANAGER_RR_SAFETY_MANAGER_RR_SAFETY_MANAGER_H_

#include "rr_common/ERP42CommandRaw.h"
#include <ros/ros.h>
#include <memory>

namespace rr_safety_manager
{
    class SafetyManager
    {
    public:
        SafetyManager();
        ~SafetyManager();
        void Init(ros::NodeHandle &nh);
        void CommandCallback(const rr_common::ERP42CommandRaw::Ptr &msg);
        void SpinOnce();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

} // namespace rr_safety_manager
#endif // RR_SAFETY_MANAGER_RR_SAFETY_MANAGER_RR_SAFETY_MANAGER_H_