#include "rr_erp42_serial_command/erp42_serial_command.h"
#include <rr_common/ERP42FeedbackRaw.h>

namespace rr_simul
{
    struct MoraiSerialConverter::Impl
    {
        ///
        //! @brief feedback msg from morai subscriber.
        ///
        ros::Subscriber sub_ego;
        ///
        //! @brief cmd msg from upper stack subscriber.
        ///
        ros::Subscriber sub_cmd;
        ///
        //! @brief feedback msg to upper stack publisher.
        ///
        ros::Publisher pub_feedback;
        ///
        //! @brief feedback_raw msg for VMS hearbeat checker publisher.
        ///
        ros::Publisher pub_feedback_raw;
        ///
        //! @brief cmd msg to morai publisher.
        ///
        ros::Publisher pub_cmd;

        ///
        //!@brief subscrived feedback msg from morai
        ///
        rr_common::ERP42FeedbackExt feedback;
        ///
        //!@brief subscrived feedback msg from morai
        ///
        rr_common::ERP42FeedbackRaw feedback_raw;
        ///
        //!@brief subscribed cmd msg from upper stack
        ///
        morai_msgs::CtrlCmd cmd_data;
        ///
        //! @brief vehicle wheel radius in meters.
        //! default is 0.2894
        ///
        float wheel_radius_in_meter;
    };

    MoraiSerialConverter::MoraiSerialConverter() : impl_(new Impl) {}
    MoraiSerialConverter::~MoraiSerialConverter() {}

    void MoraiSerialConverter::Init(ros::NodeHandle &nh)
    {
        // ready for publish
        impl_->pub_feedback = nh.advertise<rr_common::ERP42FeedbackExt>("feedback_output", 1);
        impl_->pub_feedback_raw = nh.advertise<rr_common::ERP42FeedbackRaw>("feedback_raw_output", 1);
        impl_->pub_cmd = nh.advertise<morai_msgs::CtrlCmd>("command_to_morai", 1);

        // ready for subscribe
        impl_->sub_ego = nh.subscribe("morai_ego", 1, &MoraiSerialConverter::MoraiEgoCallback, this);
        impl_->sub_cmd = nh.subscribe("morai_cmd", 1, &MoraiSerialConverter::ErpCmdCallback, this);

        //initial setting
        bool is_init = true;
        if (is_init)
        {
            impl_->feedback.header.frame_id = "rr_device";
            impl_->feedback_raw.header.frame_id = "vms";
            impl_->feedback_raw.a_or_m = 1;
            impl_->feedback_raw.gear = 0;
            is_init = false;
        }
    }

    void MoraiSerialConverter::MoraiEgoCallback(const morai_msgs::EgoVehicleStatus::Ptr &ego)
    {

        // get velocity m/s
        auto speedxyz_ = ego->velocity;

        //  add subscribed message to feedback data for publish feedback
        impl_->feedback.header.stamp = ros::Time::now();
        impl_->feedback.speed_mps = float(speedxyz_.x);
        impl_->feedback.speed_kph = float(impl_->feedback.speed_mps * 3600 / 1000);
        impl_->feedback.steer_deg = float(ego->wheel_angle);
        impl_->feedback.steer_rad = float(impl_->feedback.steer_deg * M_PI / 180.0);
        impl_->feedback.brake_ratio = float(ego->brake);

        //  add ros time now to feedback raw data for publish feedback
        impl_->feedback_raw.header.stamp = impl_->feedback.header.stamp;
        impl_->feedback_raw.steer = float(ego->wheel_angle);
        impl_->feedback_raw.speed = speedxyz_.x * 30 / (M_PI * impl_->wheel_radius_in_meter);
        impl_->feedback_raw.brake = float(ego->brake) * 100;
        if (impl_->feedback_raw.brake == 0)
        {
            impl_->feedback_raw.brake = 1;
        }
    }
    void MoraiSerialConverter::ErpCmdCallback(const rr_common::ERP42CommandRaw::Ptr &cmd)
    {
        //  add to message for publish cmd
        impl_->cmd_data.longlCmdType = (int32_t)(1);                   // way to control 1: Throttle control, 2: Velocity control, 3: Acceleration control
        impl_->cmd_data.accel = (double)(cmd->speed) / 1000;           // in -> double (0~1)
        impl_->cmd_data.steering = -(double)(cmd->steer * M_PI / 180); // int-> doubel (rad)

        // enable brake when manual mode, estop
        if (cmd->a_or_m == 0 || cmd->e_stop == 1)
        {
            impl_->cmd_data.brake = 1;
        }
        else
        {
            impl_->cmd_data.brake = double(cmd->brake - 1) / 99; // in -> double (0.0~1.0)
        }
    }

    void MoraiSerialConverter::SpinOnce()
    {
        //activate publish
        impl_->pub_feedback.publish(impl_->feedback);
        impl_->pub_feedback_raw.publish(impl_->feedback_raw);
        impl_->pub_cmd.publish(impl_->cmd_data);
    }
}