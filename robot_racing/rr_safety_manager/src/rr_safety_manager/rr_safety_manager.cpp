#include "rr_safety_manager/rr_safety_manager.h"
#include "rr_safety_manager/rr_course_out_detector.h"

namespace rr_safety_manager
{

    struct SafetyManager::Impl
    {
        // detecting algorithm module
        CourseOutDetector course_out_detector;

        // latest message sotred
        rr_common::ERP42CommandRaw msg_command_in;

        // publisher and subscribers
        ros::Publisher pub_vms_command_raw;
        ros::Subscriber sub_controller_command_raw;
    };

    SafetyManager::SafetyManager() : impl_(new Impl) {}

    SafetyManager::~SafetyManager() {}

    void SafetyManager::Init(ros::NodeHandle &nh)
    {
        impl_->pub_vms_command_raw = nh.advertise<rr_common::ERP42CommandRaw>("erp42_command_raw_out", 3);
        impl_->sub_controller_command_raw = nh.subscribe("erp42_command_raw_in", 3, &SafetyManager::CommandCallback, this);
        impl_->course_out_detector.Init(nh);

        // initial value for safety
        impl_->msg_command_in.header.stamp = ros::Time::now();
        impl_->msg_command_in.a_or_m = rr_common::ERP42CommandRaw::kModeAuto;
        impl_->msg_command_in.brake = 1;
        impl_->msg_command_in.e_stop = rr_common::ERP42CommandRaw::kEStopOn;
        impl_->msg_command_in.gear = rr_common::ERP42CommandRaw::kGearNeutral;
        impl_->msg_command_in.speed = 0;
        impl_->msg_command_in.steer = 0;
    }

    void SafetyManager::CommandCallback(const rr_common::ERP42CommandRaw::Ptr &msg)
    {
        impl_->msg_command_in = *msg;
    }

    void SafetyManager::SpinOnce()
    {
        rr_common::ERP42CommandRaw msg_override;
        msg_override = impl_->msg_command_in;
        const bool &is_out = impl_->course_out_detector.IsOut();
        ROS_FATAL_COND(is_out, "[ SAFETY_MANAGER ]: Vehicle is out of the course");
        if (is_out)
        {
            msg_override.speed = 0;                                       // no speed
            msg_override.brake = 100;                                     // full brake
            msg_override.gear = rr_common::ERP42CommandRaw::kGearNeutral; // gear N
            msg_override.e_stop = rr_common::ERP42CommandRaw::kEStopOn;   // estop on
        }
        impl_->pub_vms_command_raw.publish(msg_override);
    }

} // rr_safety_manager