#include "rr_erp42/erp42.h"
#include "rr_erp42/protocol.h"
#include "rr_serial/serial.h"
#include "rr_common/ERP42FeedbackRaw.h"
#include "rr_common/ERP42FeedbackExt.h"
#include <vector>
#include <cmath>

namespace rr_devices
{

    struct ERP42::Impl
    {
        ///
        //! @brief ERP42 serial deivce
        ///
        SerialDevice device;
        ///
        //! @brief ERP42 P2H packet reading buffer.
        ///
        std::vector<uint8_t> read_buf;
        ///
        //! @brief ERP42 H2P packet writing buffer.
        ///
        std::vector<uint8_t> write_buf;

        ///
        //! @brief P2H packet parser
        ///
        P2H packet_parser;

        ///
        //! @brief H2P packet generator
        ///
        H2P packet_generator;

        ///
        //! @brief feedback message publisher.
        ///
        ros::Publisher pub_feedback;
        ///
        //! @brief extended feedback message publisher
        ///
        ros::Publisher pub_feedback_ext;
        ///
        //! @brief command message psubscriber.
        ///
        ros::Subscriber sub_command;
        ///
        //! @brief latest arrived command message.
        ///
        rr_common::ERP42CommandRaw msg_command;
        ///
        //! @brief Extended message parameter
        //! ERP42FeedbackRaw::steer / feedback_steer_degree_to_raw = steering target degree [deg]
        //! default is (102.0)
        ///
        float feedback_steer_degree_to_raw;
        ///
        //! @brief vehicle wheel radius in meters.
        //! ERP42FeedbackRaw::speed * wheel_radius_in_meter * 2 * pi / 60 =  vehicle speed [m/s]
        //! default is 0.2894
        ///
        float wheel_radius_in_meter;
    };

    ERP42::ERP42() : impl_(new Impl) {}
    ERP42::~ERP42() {}

    void ERP42::Initialize(ros::NodeHandle &nh)
    {
        // read params
        std::string dev_path; // device path, default: /dev/ttyS0
        int baud;             // baudrate for the device, default: 115200
        nh.param<std::string>("dev_path", dev_path, "/dev/ttyS0");
        nh.param<int>("baud", baud, 115200);
        nh.param<float>("feedback_steer_degree_to_raw", impl_->feedback_steer_degree_to_raw, 102.0);
        nh.param<float>("wheel_radius_in_meter", impl_->wheel_radius_in_meter, 0.2894);

        // verbose
        ROS_INFO("ERP42 serial device path: %s", dev_path.c_str());
        ROS_INFO("ERP42 serial baudrate: %d", baud);

        // create publisher
        impl_->pub_feedback = nh.advertise<rr_common::ERP42FeedbackRaw>("feedback", 3);
        impl_->pub_feedback_ext = nh.advertise<rr_common::ERP42FeedbackExt>("feedback_ext", 3);

        impl_->sub_command = nh.subscribe("command", 3, &ERP42::CommandCallback, this);

        // prepare device
        ROS_ASSERT(impl_->device.Open(dev_path));
        ROS_ASSERT(impl_->device.Configure(baud, 8));

        // prepare buffers
        impl_->read_buf.resize(kP2HPacketSize + 1);
        impl_->packet_parser.SetData(impl_->read_buf.data());
        impl_->write_buf.resize(kH2PPacketSize);
        impl_->packet_generator.SetData(impl_->write_buf.data());
        impl_->packet_generator.SetSTXEXT();
        impl_->packet_generator.SetAlive(255);
    }

    void ERP42::CommandCallback(const rr_common::ERP42CommandRawConstPtr msg)
    {
        double delay = (ros::Time::now() - msg->header.stamp).toSec();
        // copy message
        impl_->msg_command.a_or_m = msg->a_or_m;
        impl_->msg_command.brake = msg->brake;
        impl_->msg_command.e_stop = msg->e_stop;
        impl_->msg_command.gear = msg->gear;
        impl_->msg_command.header = msg->header;
        impl_->msg_command.speed = msg->speed;
        impl_->msg_command.steer = msg->steer;
        // we copy alive for checking, but actually written alive value to the ERP42 serial device may not be same.
        // alive value should be updated at every writing, not receiving message.
        impl_->msg_command.alive = msg->alive;
    }
    bool ERP42::ReadUpdate()
    {
        // packet arrives every 50ms. can't be timeout. immediately return at reading one or more bytes.
        bool valid = false;
        for (size_t i = 0; (!valid) && (i < kP2HPacketSize * 2); ++i)
        {
            // read one byte
            int read_n = impl_->device.Read(&impl_->read_buf[kP2HPacketSize], 1);
            // read fail.
            if (read_n <= 0)
            {
                ROS_WARN("read failed");
            }
            // read success. check validity
            else
            {
                // shift the buffer one byte
                for (size_t j = 0; j < kP2HPacketSize; ++j)
                {
                    impl_->read_buf[j] = impl_->read_buf[j + 1];
                }
                valid = impl_->packet_parser.IsSTXEXT();
            }
        }

        // validation done.
        if (valid)
        {
            ros::Time pub_time = ros::Time::now();
            // copy values to the message
            rr_common::ERP42FeedbackRaw msg;
            msg.header.frame_id = "rr_device";
            msg.header.stamp = pub_time;
            msg.a_or_m = impl_->packet_parser.GetAorM();
            msg.e_stop = impl_->packet_parser.GetEStop();
            msg.gear = impl_->packet_parser.GetGear();
            msg.speed = impl_->packet_parser.GetSpeedRPM();
            msg.steer = impl_->packet_parser.GetSteer();
            msg.brake = impl_->packet_parser.GetBrake();
            msg.encoder = impl_->packet_parser.GetEncoder();
            msg.batt = impl_->packet_parser.GetBattery();
            msg.alive = impl_->packet_parser.GetAlive();

            rr_common::ERP42FeedbackExt msg_ext;
            msg_ext.header.frame_id = "rr_device";
            msg_ext.header.stamp = pub_time;
            msg_ext.brake_ratio = ((float)(msg.brake - 1)) / 99.0;
            msg_ext.speed_mps = msg.speed * M_PI * impl_->wheel_radius_in_meter / 30.0;
            msg_ext.speed_kph = msg_ext.speed_mps * 3.6;
            msg_ext.steer_deg = msg.steer / impl_->feedback_steer_degree_to_raw;
            msg_ext.steer_rad = msg_ext.steer_deg * M_PI / 180.0;

            // publish message.
            impl_->pub_feedback.publish(msg);
            impl_->pub_feedback_ext.publish(msg_ext);
        }
        else
        {
            ROS_FATAL("Problem reading pakcet. reading failed 10 times. (for 1 sec)");
        }
        return valid;
    }

    bool ERP42::WriteUpdate()
    {

        switch (impl_->msg_command.a_or_m)
        {
        case kModeAuto:
            impl_->packet_generator.SetAorM(kModeAuto);
            break;
        case kModeManual:
            impl_->packet_generator.SetAorM(kModeManual);
            break;
        default:
            ROS_FATAL("Unknown Mode. only 0 and 1 is available for AorM");
            break;
        }

        switch (impl_->msg_command.e_stop)
        {
        case kEStopOn:
            impl_->packet_generator.SetEStop(kEStopOn);
            break;
        case kEStopOff:
            impl_->packet_generator.SetEStop(kEStopOff);
            break;
        default:
            ROS_FATAL("Unknown EStop. only 0 and 1 is available for EStop");
            break;
        }
        switch (impl_->msg_command.gear)
        {
        case kGearForward:
            impl_->packet_generator.SetGear(kGearForward);
            break;
        case kGearBackward:
            impl_->packet_generator.SetGear(kGearBackward);
            break;
        case kGearNeutral:
            impl_->packet_generator.SetGear(kGearNeutral);
            break;
        default:
            ROS_FATAL("Unknown Gear. only 0,1, and 2 is available for Gear");
            break;
        }

        impl_->packet_generator.SetSpeedMotorRaw(impl_->msg_command.speed);
        impl_->packet_generator.SetSteer(impl_->msg_command.steer);
        impl_->packet_generator.SetBrake(impl_->msg_command.brake);
        impl_->packet_generator.UpdateAlive();

        ssize_t write_n = impl_->device.Write(impl_->write_buf.data(), kH2PPacketSize);
        return write_n > 0;
    }

    void ERP42::SpinOnce()
    {
        // reply command only after successful reading
        if (!ReadUpdate())
        {
            ROS_FATAL("reading from ERP42 failed");
        }
        if (!WriteUpdate())
        {
            ROS_FATAL("writing to ERP42 failed");
        }
    }

}