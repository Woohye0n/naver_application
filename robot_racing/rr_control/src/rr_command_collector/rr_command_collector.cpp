#include "rr_command_collector/rr_command_collector.h"
#include "rr_common/ERP42CommandRaw.h"

namespace rr_control
{
    struct ControlCommandCollector::Impl
    {
        ros::Publisher pub_raw;

        ros::Subscriber sub_speed;
        ros::Subscriber sub_steer;
        ros::Timer timer;

        rr_common::ERP42CommandRaw msg_command_raw;
    };

    ControlCommandCollector::ControlCommandCollector() : impl_(new Impl) {}
    ControlCommandCollector::~ControlCommandCollector() {}

    void ControlCommandCollector::Init(ros::NodeHandle &nh)
    {
        impl_->sub_speed = nh.subscribe("speed_command_in", 3, &ControlCommandCollector::SpeedCommandCallback, this);
        impl_->sub_steer = nh.subscribe("steer_command_in", 3, &ControlCommandCollector::SteerCommandCallback, this);
        impl_->pub_raw = nh.advertise<rr_common::ERP42CommandRaw>("raw_command_out", 3);

        // period is set to 0.025 sec (40hz)
        impl_->timer = nh.createTimer(ros::Duration(0.025), &ControlCommandCollector::TimerCallback, this);

        // initial message setup
        impl_->msg_command_raw.header.stamp = ros::Time::now();
        impl_->msg_command_raw.header.frame_id = "command_collector";
        impl_->msg_command_raw.a_or_m = rr_common::ERP42CommandRaw::kModeAuto;
        impl_->msg_command_raw.alive = 0;
        impl_->msg_command_raw.brake = 80;
        impl_->msg_command_raw.e_stop = rr_common::ERP42CommandRaw::kEStopOff;
        impl_->msg_command_raw.gear = rr_common::ERP42CommandRaw::kGearNeutral;
        impl_->msg_command_raw.speed = 0;
        impl_->msg_command_raw.steer = 0;
    }

    void ControlCommandCollector::SpeedCommandCallback(const rr_common::SpeedControlCommand::Ptr &msg)
    {
        impl_->msg_command_raw.speed = msg->speed;
        impl_->msg_command_raw.gear = msg->gear;
        impl_->msg_command_raw.brake = msg->brake;
    }

    void ControlCommandCollector::SteerCommandCallback(const rr_common::SteerControlCommand::Ptr &msg)
    {
        // change deg to steer (0-2000)
        impl_->msg_command_raw.steer = msg->steer * 102;
    }

    void ControlCommandCollector::TimerCallback(const ros::TimerEvent &event)
    {
        impl_->msg_command_raw.header.stamp = event.current_real;
        impl_->pub_raw.publish(impl_->msg_command_raw);
    }

} // namespace rr_control