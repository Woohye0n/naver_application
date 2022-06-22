#include "rr_state_estimation/rr_state_estimation.h"

namespace rr_state_estimation
{
    struct StateEstimation::Impl
    {
        ros::ServiceClient client;
        ros::Subscriber sub_gps;
        ros::Subscriber sub_erp;
        ros::Publisher pub_state;

        // variables
        float heading;
        rr_common::StateEstimated state;

        ///
        //! @brief save information from service
        ///
        rr_common::PointGps origin; //[lat,lon]

        ///
        //! @brief current pose in xy and gps coordinate
        ///
        rr_common::PointXY cur_xy; //[x,y]
    };

    StateEstimation::StateEstimation() : impl_(new Impl) {}
    StateEstimation::~StateEstimation() {}

    void StateEstimation::Init(ros::NodeHandle &nh)
    {
        // request service
        impl_->client = nh.serviceClient<rr_common::LapPoint>("/gps_lane_points/lap_point");
        rr_common::LapPoint srv;
        srv.request.header.stamp = ros::Time::now();

        // wait until response arrive
        bool init = true;
        ROS_INFO("Waiting for service response");

        while (init)
        {
            if (impl_->client.call(srv))
            {
                impl_->origin = srv.response.way_point_origin;
                init = false;
                ROS_INFO("State Estimation Service Recieved");
            }
        }
        impl_->pub_state = nh.advertise<rr_common::StateEstimated>("state", 3);
        impl_->sub_erp = nh.subscribe("feedback", 1, &StateEstimation::ERPCallback, this);
        impl_->sub_gps = nh.subscribe("/ublox_gps/navpvt", 1, &StateEstimation::GpsCallback, this);
    }

    void StateEstimation::GpsCallback(const ublox_msgs::NavPVT::Ptr &gps)
    {
        // change current gps coordinate to xy coordinate
        rr_common::PointGps cur_gps;

        cur_gps.lon = gps->lon * pow(10, (-7));
        cur_gps.lat = gps->lat * pow(10, (-7));

        ConvertGps2XY(cur_gps);

        // get heading
        impl_->heading = gps->heading * pow(10, (-5));
        if (impl_->heading > 180)
        {
            impl_->heading -= 360;
        }

        // add to message
        impl_->state.current_xy = impl_->cur_xy;
        impl_->state.heading = impl_->heading;

        // publish
        impl_->pub_state.publish(impl_->state);
    }

    void StateEstimation::ERPCallback(const rr_common::ERP42FeedbackExt::Ptr &feedback)
    {
        // get speed from feedback
        impl_->state.speed = feedback->speed_kph;
        impl_->state.angular_velocity = -feedback->speed_mps * (tan(feedback->steer_rad) / 1.212);
        impl_->state.header.stamp = feedback->header.stamp;
    }

    void StateEstimation::ConvertGps2XY(const rr_common::PointGps &gps)
    {
        // converting current position coordinate gps to xy
        double longitude = gps.lon - impl_->origin.lon;
        double latitude = gps.lat - impl_->origin.lat;

        float distance_x = longitude * cos(impl_->origin.lat * M_PI / 180) * M_PI * 6378.135 / 180 * 1000;
        float distance_y = latitude * M_PI * 6378.135 / 180 * 1000;
        impl_->cur_xy.x = distance_x;
        impl_->cur_xy.y = distance_y;
    }
}
