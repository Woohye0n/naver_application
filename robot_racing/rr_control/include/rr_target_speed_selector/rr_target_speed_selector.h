#ifndef RR_CONTROL_RR_TARGET_SPEED_SELECTOR_RR_TARGET_SPEED_SELECTOR_H_
#define RR_CONTROL_RR_TARGET_SPEED_SELECTOR_RR_TARGET_SPEED_SELECTOR_H_

#include "rr_common/SpeedControlTarget.h"
#include <ros/ros.h>
#include <memory>

namespace rr_control
{
    class TargetSpeedSelector
    {
    public:
        TargetSpeedSelector();
        ~TargetSpeedSelector();
        void Init(ros::NodeHandle &nh);
        void TargetCandidateCallback(const rr_common::SpeedControlTarget::Ptr &msg);
        void TimerCallback(const ros::TimerEvent &event);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
} // namespace rr_control

#endif