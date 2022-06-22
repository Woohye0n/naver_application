#include "rr_map/rr_map.h"

namespace rr_map
{
    struct Map::Impl
    {
        rr_common::PointGps centroid;

        ///
        //! @brief vector data for service request(xy points)
        ///
        std::vector<rr_common::PointXY> racing_xy;
        std::vector<rr_common::PointXY> highway_in_xy;
        std::vector<rr_common::PointXY> highway_out_xy;

        ///
        //! @brief client for get map gps way points
        //! @brief server for sevice map xy wat points
        ///
        ros::ServiceClient client;
        ros::ServiceServer service_map_xy;
    };

    Map::Map() : impl_(new Impl) {}
    Map::~Map() {}

    void Map::Init(ros::NodeHandle &nh)
    {
        ///
        //! @brief request GpsTrackData
        ///
        impl_->client = nh.serviceClient<rr_common::GpsTrackData>("/gps_lane_points/gpstrackdata");
        rr_common::GpsTrackData track_data;
        track_data.request.header.stamp = ros::Time::now();
        bool init = true;

        ROS_INFO("Waiting for service response");

        while (init)
        {
            if (impl_->client.call(track_data))
            {
                impl_->centroid = track_data.response.centroid;

                SaveVector(track_data.response.highway_in, impl_->highway_in_xy);
                SaveVector(track_data.response.highway_out, impl_->highway_out_xy);
                SaveVector(track_data.response.racing_point, impl_->racing_xy);

                init = false;
                ROS_INFO("Service Recieved");
            }
        }

        impl_->service_map_xy = nh.advertiseService("/map_xy_points", &Map::AddMapXYSrv, this);
    }

    bool Map::AddMapXYSrv(rr_common::MapWayPointXY::Request &req, rr_common::MapWayPointXY::Response &res)
    {
        ///
        //! @brief add data into MapPointXY service
        ///
        res.racing = impl_->racing_xy;
        res.highway_in = impl_->highway_in_xy;
        res.highway_out = impl_->highway_out_xy;
        return true;
    }

    void Map::SaveVector(std::vector<rr_common::PointGps> &gps, std::vector<rr_common::PointXY> &dst)
    {
        ///
        //! @brief convert gps to xy and save in vector. [x,y] order
        ///
        auto it = gps.begin();
        auto end = gps.end();

        dst.clear();

        while (it != end)
        {
            dst.push_back(ConvertGps2XY(*it));
            it++;
        }
    }

    // gps to xy convert 함수
    rr_common::PointXY Map::ConvertGps2XY(rr_common::PointGps p_gps)
    {
        rr_common::PointXY p_xy;
        double latitude = p_gps.lat - impl_->centroid.lat;
        double longitude = p_gps.lon - impl_->centroid.lon;
        p_xy.x = longitude * cos(impl_->centroid.lat * M_PI / 180) * M_PI * 6378.135 / 180 * 1000;
        p_xy.y = latitude * M_PI * 6378.135 / 180 * 1000;

        return p_xy;
    }
}
