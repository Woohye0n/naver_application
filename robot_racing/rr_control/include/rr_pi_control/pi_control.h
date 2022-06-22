#ifndef RR_CONTROL_RR_PI_CONTROL_PI_CONTROL_H_
#define RR_CONTROL_RR_PI_CONTROL_PI_CONTROL_H_

#include "rr_common/SpeedControlTarget.h"
#include "rr_common/PIControllerOnOff.h"
#include "rr_common/StateEstimated.h"
#include "rr_common/SpeedControlCommand.h"

#include <ros/ros.h>
#include <cmath>
#include <memory>

namespace rr_control
{
    ///
    //! @brief PI control system for speed command
    ///
    class PIController
    {
    public:
        PIController();
        ~PIController();
        void Init(ros::NodeHandle &nh);
        void FeedbackCallback(const rr_common::StateEstimated::Ptr &msg);
        void TargetCallback(const rr_common::SpeedControlTarget::Ptr &msg);
        void Reset();
        bool OnOffService(rr_common::PIControllerOnOff::Request &req, rr_common::PIControllerOnOff::Response &res);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif