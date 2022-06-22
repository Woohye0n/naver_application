#include "rr_tf_maker/tf_maker.h"
#include <cmath>

namespace rr_tf_maker
{

    struct TFMaker::Impl
    {
        ///
        //!@brief
        ///
        ros::Subscriber sub_car_point;

        rr_common::StateEstimated point;

        tf::TransformBroadcaster br;

        tf::Transform base_link_transform;
        tf::Transform lidar_transform;

        ///
        //!@brief parameter about sensor position (unit:m)
        ///

        // GPS position
        float gps_z;

        // velodyne position base on GPS
        float velodyne_x;
        float velodyne_y;
        float velodyne_z;
    };

    TFMaker::TFMaker() : impl_(new Impl) {}
    TFMaker::~TFMaker() {}

    void TFMaker::Init(ros::NodeHandle &nh)
    {
        // get param about position of gps
        ROS_ASSERT(nh.getParam("gps_z", impl_->gps_z));

        // get param about position of velodyne from base_link
        ROS_ASSERT(nh.getParam("velodyne_x", impl_->velodyne_x));
        ROS_ASSERT(nh.getParam("velodyne_y", impl_->velodyne_y));
        ROS_ASSERT(nh.getParam("velodyne_z", impl_->velodyne_z));

        // ready for subscribe
        impl_->sub_car_point = nh.subscribe("StateEstimated", 1, &TFMaker::CarPositionCallback, this);
    }

    void TFMaker::CarPositionCallback(const rr_common::StateEstimated::Ptr &car)
    {
        // set base_link tf information
        // set position of base_link as GPS position on car (unit: m)
        impl_->base_link_transform.setOrigin(tf::Vector3(car->current_xy.x, car->current_xy.y, impl_->gps_z));

        // set rotation of car according to heading from gps (unit: rad)
        tf::Quaternion q;
        q.setRPY(0, 0, (-car->heading + 90) * M_PI / 180);
        impl_->base_link_transform.setRotation(q);
    }

    void TFMaker::SpinOnce()
    {
        // setting velodyne tf information
        // set position of velodyne from base_link (unit: m)
        impl_->lidar_transform.setOrigin(tf::Vector3(impl_->velodyne_x, impl_->velodyne_y, impl_->velodyne_z));
        // set rotation of velodyne from base_link (unit: rad)
        impl_->lidar_transform.setRotation(tf::Quaternion(0, 0, 0, 1));

        // broadcasting tf
        // tf about map to base_link
        impl_->br.sendTransform(tf::StampedTransform(impl_->base_link_transform, ros::Time::now(), "map", "base_link"));
        // tf about base_link to velodyne
        impl_->br.sendTransform(tf::StampedTransform(impl_->lidar_transform, ros::Time::now(), "base_link", "velodyne"));
    }
}
