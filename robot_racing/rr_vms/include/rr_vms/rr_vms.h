#ifndef RR_VMS_RR_VMS_RR_VMS_H_
#define RR_VMS_RR_VMS_RR_VMS_H_

#include "rr_common/ERP42FeedbackRaw.h"
#include "rr_common/ERP42CommandRaw.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <ublox_msgs/NavPVT.h>

#include <ros/ros.h>
#include <memory>

namespace rr_vms
{
    class VMS
    {
    public:
        VMS();
        ~VMS();
        void Init(ros::NodeHandle &nh);
        void ERP42CommandCallback(const rr_common::ERP42CommandRaw::Ptr &msg);
        void ERP42FeedbackCallback(const rr_common::ERP42FeedbackRaw::Ptr &msg);
        void VelodyneCallback(const sensor_msgs::PointCloud2::Ptr &msg);
        void CameraCallback(const sensor_msgs::Image::Ptr &msg);
        void GPSCallback(const ublox_msgs::NavPVT::Ptr &msg);
        void SpinOnce();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}

#endif // RR_VMS_RR_VMS_RR_VMS_H_