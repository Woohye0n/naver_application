#include "rr_pi_control/pi_control.h"

namespace rr_control
{

    const float &Min(const float &a, const float &b)
    {
        return a < b ? a : b;
    }

    const float &Max(const float &a, const float &b)
    {
        return a > b ? a : b;
    }
    struct PIController::Impl
    {
        ros::Publisher pub_command;

        ros::Subscriber sub_target;
        ros::Subscriber sub_feedback;

        ros::ServiceServer srv_on_off;

        // P and I
        float P = 0;
        float I = 0;

        // control gains for brake
        float brake_k_p;

        // control gains for motor
        float motor_k_p;
        float motor_k_i;

        // control target speed in kph
        float target_speed;

        // previous update log
        ros::Time last_update_timestamp;

        // pi controller on off switch
        bool is_on;

        // braking_threshold to use brake
        float braking_threshold;

        // message buffer
        rr_common::SpeedControlCommand msg_command_out;
    };

    PIController::PIController() : impl_(new Impl) {}

    PIController::~PIController() {}

    void PIController::Init(ros::NodeHandle &nh)
    {
        // read ros param
        ROS_ASSERT(nh.getParam("motor_k_p", impl_->motor_k_p));
        ROS_ASSERT(nh.getParam("motor_k_i", impl_->motor_k_i));
        ROS_ASSERT(nh.getParam("brake_k_p", impl_->brake_k_p));
        ROS_ASSERT(nh.getParam("braking_threshold", impl_->braking_threshold));

        // setup publisher
        impl_->pub_command = nh.advertise<rr_common::SpeedControlCommand>("command", 3);

        // setup subscribers
        impl_->sub_target = nh.subscribe("target_speed", 1, &PIController::TargetCallback, this);
        impl_->sub_feedback = nh.subscribe("feedback", 1, &PIController::FeedbackCallback, this);

        // setup on-off service
        impl_->srv_on_off = nh.advertiseService("on_off", &PIController::OnOffService, this);

        // reset controller
        Reset();

        // off by default
        impl_->is_on = false;
    }

    void PIController::Reset()
    {
        // reset the pi controller not to make the control error explode.
        impl_->last_update_timestamp = ros::Time::now();
        impl_->P = 0;
        impl_->I = 0;
    }

    void PIController::FeedbackCallback(const rr_common::StateEstimated::Ptr &msg)
    {
        if (!impl_->is_on)
        {
            ROS_DEBUG_DELAYED_THROTTLE(1.0, "[ CONTROLLER ]: pi controller is OFF. Request to turn on the controller.");
            return;
        }
        // update error
        float err = impl_->target_speed - msg->speed;
        float dt = (msg->header.stamp - impl_->last_update_timestamp).toSec();

        // update header and timestamp
        impl_->msg_command_out.header.stamp = msg->header.stamp;
        impl_->last_update_timestamp = msg->header.stamp;

        // update input
        uint8_t gear_in = rr_common::SpeedControlCommand::kGearForward;
        uint16_t speed_in;
        uint8_t brake_in;
        float speed_in_tmp;

        impl_->P = impl_->motor_k_p * err;
        impl_->I = impl_->motor_k_i * err * dt;
        speed_in_tmp = (impl_->P + impl_->I < 0) ? 0 : impl_->P + impl_->I;
        speed_in = speed_in_tmp;

        if (err < -impl_->braking_threshold)
        {
            speed_in = 0;
            brake_in = impl_->brake_k_p * err;
        }

        // protocol limit
        speed_in = Max(Min(speed_in, 1000), 0);
        brake_in = Max(Min(brake_in, 100), 1);

        // set message
        impl_->msg_command_out.gear = gear_in;
        impl_->msg_command_out.speed = speed_in;
        impl_->msg_command_out.brake = brake_in;

        // publish message
        impl_->pub_command.publish(impl_->msg_command_out);
    }

    void PIController::TargetCallback(const rr_common::SpeedControlTarget::Ptr &msg)
    {
        impl_->target_speed = msg->speed;
    }

    bool PIController::OnOffService(rr_common::PIControllerOnOff::Request &req, rr_common::PIControllerOnOff::Response &res)
    {
        if (req.is_on == req.kOn)
        {
            Reset();
            impl_->is_on = true;
            ROS_INFO("[ CONTROLLER ]: pi controller is ON.");
        }
        else
        {
            Reset();
            impl_->is_on = false;
            ROS_INFO("[ CONTROLLER ]: pi controller is OFF.");
        }
        return true;
    }

}