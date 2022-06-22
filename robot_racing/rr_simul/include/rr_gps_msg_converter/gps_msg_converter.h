#ifndef RR_SIMUL_RR_GPS_MSG_CONVERTER_GPS_MSG_CONVERTER_H_
#define RR_SIMUL_RR_GPS_MSG_CONVERTER_GPS_MSG_CONVERTER_H_

#include <ublox_msgs/NavPVT.h>
#include <morai_msgs/GPSMessage.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <ros/ros.h>

namespace rr_simul
{
    class MoraiGpsConverter
    {
    public:
        MoraiGpsConverter();
        ~MoraiGpsConverter();

        void Init(ros::NodeHandle &nh);
        void MoraiGpsCallback(const morai_msgs::GPSMessage::Ptr &gps);
        void MoraiEgoCallback(const morai_msgs::EgoVehicleStatus::Ptr &ego);
        void SpinOnce();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // RR_SIMUL_RR_GPS_MSG_CONVERTER_GPS_MSG_CONVERTER_H_
