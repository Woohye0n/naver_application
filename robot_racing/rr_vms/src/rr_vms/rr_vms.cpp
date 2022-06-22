#include "rr_vms/rr_vms.h"
#include "rr_vms/heartbeat_checker.h"
#include "rr_vms/gps_status_checker.h"
#include "rr_vms/drive_mode_checker.h"

namespace rr_vms
{
    struct VMS::Impl
    {
        // latest message for checking
        rr_common::ERP42CommandRaw msg_erp42_command_raw_in; // command from safety manager

        // publisher and subscribers
        ros::Publisher pub_erp42_command_raw; // command from safety manager is forwarded by this publisher

        ros::Subscriber sub_erp42_command_raw;  // subscriber for raw command from rr_safety_manager
        ros::Subscriber sub_erp42_feedback_raw; // subscriber for feedback from rr_erp42
        ros::Subscriber sub_gps;                // subscriber for message from gps
        ros::Subscriber sub_velodyne;           // subscriber for velodyne point clouds
        ros::Subscriber sub_cam;                // subscriber for camera image

        HearbeatChecker hb_erp42;            // heartbeat checker for rr_erp42
        HearbeatChecker hb_velodyne;         // heartbeat checker for gps
        HearbeatChecker hb_cam;              // heartbeat checker for velodyne
        HearbeatChecker hb_gps;              // heartbeat checker for camera
        GPSStatusChecker gps_status_checker; // gps status checker. fix or not.
        DriveModeChecker drive_mode_checker; // check if the mdoe is auto
    };

    VMS::VMS() : impl_(new Impl) {}
    VMS::~VMS() {}

    void VMS::Init(ros::NodeHandle &nh)
    {
        // setting up publishers
        impl_->pub_erp42_command_raw = nh.advertise<rr_common::ERP42CommandRaw>("erp42_command_raw_out", 3);

        // setting up subscribers
        impl_->sub_erp42_command_raw = nh.subscribe("erp42_command_raw_in", 3, &VMS::ERP42CommandCallback, this);
        impl_->sub_erp42_feedback_raw = nh.subscribe("erp42_feedback_raw", 3, &VMS::ERP42FeedbackCallback, this);
        impl_->sub_gps = nh.subscribe("gps", 3, &VMS::GPSCallback, this);
        impl_->sub_velodyne = nh.subscribe("velodyne_points", 3, &VMS::VelodyneCallback, this);
        impl_->sub_cam = nh.subscribe("camera", 3, &VMS::CameraCallback, this);

        // setting up heartbeat checkers.
        // you should setup these parameters in launch file.
        impl_->hb_erp42.Init(nh, "erp42_feedback_hb_interval", "erp42_feedback_hb_jitter", "erp42");
        impl_->hb_velodyne.Init(nh, "velodyne_hb_interval", "velodyne_hb_jitter", "velodyne");
        impl_->hb_cam.Init(nh, "camera_hb_interval", "camera_hb_jitter", "cam");
        impl_->hb_gps.Init(nh, "gps_hb_interval", "gps_hb_jitter", "gps");
        impl_->gps_status_checker.Init(nh);
        impl_->drive_mode_checker.Init(nh);

        // initial value for safety
        impl_->msg_erp42_command_raw_in.header.stamp = ros::Time::now();
        impl_->msg_erp42_command_raw_in.a_or_m = rr_common::ERP42CommandRaw::kModeAuto;
        impl_->msg_erp42_command_raw_in.brake = 1;
        impl_->msg_erp42_command_raw_in.e_stop = rr_common::ERP42CommandRaw::kEStopOn;
        impl_->msg_erp42_command_raw_in.gear = rr_common::ERP42CommandRaw::kGearNeutral;
        impl_->msg_erp42_command_raw_in.speed = 0;
        impl_->msg_erp42_command_raw_in.steer = 0;
    }
    void VMS::ERP42CommandCallback(const rr_common::ERP42CommandRaw::Ptr &msg)
    {
        impl_->msg_erp42_command_raw_in = *msg;
    }
    void VMS::ERP42FeedbackCallback(const rr_common::ERP42FeedbackRaw::Ptr &msg)
    {
        // don't need to copy all.
        impl_->hb_erp42.SetStamp(ros::Time::now());
        impl_->drive_mode_checker.CheckMessage(msg);
    }

    void VMS::VelodyneCallback(const sensor_msgs::PointCloud2::Ptr &msg)
    {
        // don't need to copy all.
        impl_->hb_velodyne.SetStamp(ros::Time::now());
    }
    void VMS::CameraCallback(const sensor_msgs::Image::Ptr &msg)
    {
        // don't need to copy all.
        impl_->hb_cam.SetStamp(ros::Time::now());
    }
    void VMS::GPSCallback(const ublox_msgs::NavPVT::Ptr &msg)
    {
        // there is no header for this message. just use ros::Time::now()
        impl_->hb_gps.SetStamp(ros::Time::now());
        impl_->gps_status_checker.CheckMessage(msg);
    }
    void VMS::SpinOnce()
    {
        const bool &is_ok_erp42 = impl_->hb_erp42.IsOk();
        const bool &is_ok_velodyne = impl_->hb_velodyne.IsOk();
        const bool &is_ok_cam = impl_->hb_cam.IsOk();
        const bool &is_ok_gps_hb = impl_->hb_gps.IsOk();
        const bool &is_ok_gps_status = impl_->gps_status_checker.IsOk();
        const bool &is_ok_drive_mode = impl_->drive_mode_checker.IsOk();

        ROS_FATAL_COND(!is_ok_erp42, "[ VMS ]: erp42 heartbeat lost ( age: %f )", impl_->hb_erp42.GetAge());
        ROS_FATAL_COND(!is_ok_velodyne, "[ VMS ]: velodyne heartbeat lost ( age: %f )", impl_->hb_velodyne.GetAge());
        //ROS_FATAL_COND(!is_ok_cam, "[ VMS ]: camera heartbeat lost ( age: %f )", impl_->hb_cam.GetAge());
        ROS_FATAL_COND(!is_ok_gps_hb, "[ VMS ]: gps heartbeat lost ( age: %f )", impl_->hb_gps.GetAge());
        ROS_WARN_COND(!is_ok_gps_status, "[ VMS ]: gps status not fix");
        ROS_WARN_COND(!is_ok_drive_mode, "[ VMS ]: drive mode is not Auto (current : %s)", impl_->drive_mode_checker.ModeString().c_str());

        const bool &is_ok_all = is_ok_erp42 && is_ok_velodyne &&
                                 is_ok_gps_hb &&
                                is_ok_gps_status && is_ok_drive_mode;

        rr_common::ERP42CommandRaw msg_override;
        msg_override = impl_->msg_erp42_command_raw_in;
        if (!is_ok_all)
        {
            msg_override.speed = 0;                                       // no speed
            msg_override.brake = 100;                                     // full brake
            msg_override.gear = rr_common::ERP42CommandRaw::kGearNeutral; // gear N
            msg_override.e_stop = rr_common::ERP42CommandRaw::kEStopOn;   // estop on
        }
        impl_->pub_erp42_command_raw.publish(msg_override);
    }
} // namespace rr_vms
