#include "rr_target_speed_selector/rr_target_speed_selector.h"
#include <vector>

namespace rr_control
{
    struct TargetSpeedSelector::Impl
    {
        ros::Publisher pub_target;  // publish a target msg to pi controller
        ros::Subscriber sub_target; // subscribe target msg from any nodes
        ros::Timer timer;

        std::vector<rr_common::SpeedControlTarget> target_speeds; // target speeds from the other nodes

        float maximum_vel;
    };

    TargetSpeedSelector::TargetSpeedSelector() : impl_(new Impl) {}
    TargetSpeedSelector::~TargetSpeedSelector() {}
    void TargetSpeedSelector::Init(ros::NodeHandle &nh)
    {
        ROS_ASSERT(nh.getParam("maximum_velocity", impl_->maximum_vel));
        impl_->pub_target = nh.advertise<rr_common::SpeedControlTarget>("pi_control_target", 3);
        impl_->sub_target = nh.subscribe("target_speed_candidate", 3, &TargetSpeedSelector::TargetCandidateCallback, this);
        rr_common::SpeedControlTarget default_msg;
        default_msg.header.stamp = ros::Time::now();
        default_msg.header.frame_id = "default";
        default_msg.speed = impl_->maximum_vel;
        impl_->target_speeds.push_back(default_msg);

        //// period is set to 0.025 sec (40hz)
        impl_->timer = nh.createTimer(ros::Duration(0.025), &TargetSpeedSelector::TimerCallback, this);
    }
    void TargetSpeedSelector::TargetCandidateCallback(const rr_common::SpeedControlTarget::Ptr &msg)
    {
        bool found = false;
        for (auto &p : impl_->target_speeds)
        {
            // if target exists with the same frame_id,
            if (p.header.frame_id == msg->header.frame_id)
            {
                found = true;
                // if incomming message is newer, update the message
                if (p.header.stamp < msg->header.stamp)
                {
                    p = *msg;
                    break;
                }
            }
        }
        // this frame id is never arrived at all. create new one.
        if (!found)
        {
            impl_->target_speeds.push_back(*msg);
        }
    }
    void TargetSpeedSelector::TimerCallback(const ros::TimerEvent &event)
    {
        // initialize
        rr_common::SpeedControlTarget msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "target_speed_selector";
        msg.speed = 0;

        // no target speed available.
        if (impl_->target_speeds.empty())
        {
            impl_->pub_target.publish(msg);
            return;
        }

        msg.speed = impl_->target_speeds[0].speed;
        for (int i = 1; i < impl_->target_speeds.size(); ++i)
        {
            if (msg.speed > impl_->target_speeds[i].speed)
            {
                msg = impl_->target_speeds[i];
            }
        }
        impl_->pub_target.publish(msg);
    }

} // namespace rr_control