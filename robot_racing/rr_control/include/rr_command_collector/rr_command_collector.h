#ifndef RR_CONTROL_RR_COMMAND_COLLECTOR_RR_COMMAND_COLLECTOR_H_
#define RR_CONTROL_RR_COMMAND_COLLECTOR_RR_COMMAND_COLLECTOR_H_

#include "rr_common/SpeedControlCommand.h"
#include "rr_common/SteerControlCommand.h"
#include <ros/ros.h>
#include <memory>

namespace rr_control
{
    class ControlCommandCollector
    {
    public:
        ControlCommandCollector();
        ~ControlCommandCollector();
        void Init(ros::NodeHandle &nh);
        void SpeedCommandCallback(const rr_common::SpeedControlCommand::Ptr &msg);
        void SteerCommandCallback(const rr_common::SteerControlCommand::Ptr &msg);
        void TimerCallback(const ros::TimerEvent &event);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
} // namespace rr_control

#endif // RR_CONTROL_RR_COMMAND_COLLECTOR_RR_COMMAND_COLLECTOR_H_
