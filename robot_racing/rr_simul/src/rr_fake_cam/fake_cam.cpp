#include "rr_fake_cam/fake_cam.h"

namespace rr_simul
{
    struct MoraiCamConverter::Impl
    {
        ///
        //!@brief publisher for cam msg to VMS
        ///
        ros::Publisher pub_fake_cam;

        ///
        //!@brief data to publish to vms
        ///
        sensor_msgs::Image fake_image;
    };

    MoraiCamConverter::MoraiCamConverter() : impl_(new Impl) {}
    MoraiCamConverter::~MoraiCamConverter() {}

    void MoraiCamConverter::Init(ros::NodeHandle &nh)
    {
        // ready for publish
        impl_->pub_fake_cam = nh.advertise<sensor_msgs::Image>("camera", 1);
    }

    void MoraiCamConverter::SpinOnce()
    {
        impl_->fake_image.header.stamp = ros::Time::now();
        impl_->pub_fake_cam.publish(impl_->fake_image);
    }
}