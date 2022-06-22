#include "rr_gps_msg_converter/gps_msg_converter.h"

namespace rr_simul
{
    struct MoraiGpsConverter::Impl
    {
        ///
        //! @brief subscribe gps data from morai
        ///
        ros::Subscriber sub_morai_gps;
        ///
        //! @brief subscribe car status(heading) from morai
        ///
        ros::Subscriber sub_morai_ego;
        ///
        //! @brief publish gps data
        ///
        ros::Publisher pub_ublox_gps;
        ///
        //!@brief subscribed gps data and heading from morai
        ///
        ublox_msgs::NavPVT ublox_msg;
    };

    MoraiGpsConverter::MoraiGpsConverter() : impl_(new Impl) {}
    MoraiGpsConverter::~MoraiGpsConverter() {}

    void MoraiGpsConverter::Init(ros::NodeHandle &nh)
    {
        // ready for publish
        impl_->pub_ublox_gps = nh.advertise<ublox_msgs::NavPVT>("ublox_gps", 1);

        // ready for subscribe
        impl_->sub_morai_gps = nh.subscribe("morai_gps", 1, &MoraiGpsConverter::MoraiGpsCallback, this);
        impl_->sub_morai_ego = nh.subscribe("morai_ego", 1, &MoraiGpsConverter::MoraiEgoCallback, this);
    }

    void MoraiGpsConverter::MoraiGpsCallback(const morai_msgs::GPSMessage::Ptr &gps)
    {
        // get gps lon, lat heading data from morai
        impl_->ublox_msg.flags = (uint8_t)(131);
        impl_->ublox_msg.lon = (int32_t)(gps->longitude * pow(10, 7));
        impl_->ublox_msg.lat = (int32_t)(gps->latitude * pow(10, 7));
    }

    void MoraiGpsConverter::MoraiEgoCallback(const morai_msgs::EgoVehicleStatus::Ptr &ego)
    {
        // get heading
        impl_->ublox_msg.heading = int((90 - ego->heading) * pow(10, 5));
    }

    void MoraiGpsConverter::SpinOnce()
    {
        impl_->pub_ublox_gps.publish(impl_->ublox_msg);
    }
}