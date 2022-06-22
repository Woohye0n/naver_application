#include "rr_data_resource_manager/rr_data_resource_manager.h"
#include <fstream>
#include <cmath>

namespace rr_data_resource_manager
{
    struct DataResourceManager::Impl
    {
        ///
        //! @brief service server
        ///
        ros::ServiceServer service_map;
        ros::ServiceServer service_ES;

        ///
        //! @brief text file path of waypoints
        ///
        std::string path_highway_in;
        std::string path_highway_out;
        std::string path_racing_point;

        ///
        //! @brief vector data for /gps_lane_points/gpstrackdata service response
        ///
        std::vector<rr_common::PointGps> readed_highway_in;
        std::vector<rr_common::PointGps> readed_highway_out;
        std::vector<rr_common::PointGps> readed_racing_point;

        ///
        //! @brief vector data for /gps_lane_points/lap_point service response
        ///
        rr_common::PointGps gps_origin_point;
        rr_common::PointGps lap_check_start_point;
        rr_common::PointGps lap_check_end_point;
    };

    DataResourceManager::DataResourceManager() : impl_(new Impl) {}
    DataResourceManager::~DataResourceManager() {}

    void DataResourceManager::Init(ros::NodeHandle &nh)
    {
        ROS_ASSERT(nh.getParam("path_highway_in", impl_->path_highway_in));
        ROS_ASSERT(nh.getParam("path_highway_out", impl_->path_highway_out));
        ROS_ASSERT(nh.getParam("path_racing_point", impl_->path_racing_point));
        ROS_ASSERT(nh.getParam("lap_check_in_point_lon", impl_->lap_check_start_point.lon));
        ROS_ASSERT(nh.getParam("lap_check_in_point_lat", impl_->lap_check_start_point.lat));
        ROS_ASSERT(nh.getParam("lap_check_out_point_lon", impl_->lap_check_end_point.lon));
        ROS_ASSERT(nh.getParam("lap_check_out_point_lat", impl_->lap_check_end_point.lat));

        ReadPoints(impl_->path_highway_in, impl_->readed_highway_in);
        ReadPoints(impl_->path_highway_out, impl_->readed_highway_out);
        ReadPoints(impl_->path_racing_point, impl_->readed_racing_point);

        CalculateCentroidOfTrackIn();

        impl_->service_map = nh.advertiseService("/gps_lane_points/gpstrackdata", &DataResourceManager::AddMapSrv, this);
        impl_->service_ES = nh.advertiseService("/gps_lane_points/lap_point", &DataResourceManager::AddSESrv, this);
    }

    bool DataResourceManager::AddMapSrv(rr_common::GpsTrackData::Request &req, rr_common::GpsTrackData::Response &res)
    {
        ROS_WARN("callback started for node map");
        res.highway_in = impl_->readed_highway_in;
        res.highway_out = impl_->readed_highway_out;
        res.racing_point = impl_->readed_racing_point;
        res.centroid = impl_->gps_origin_point;
        return true;
    }

    bool DataResourceManager::AddSESrv(rr_common::LapPoint::Request &req, rr_common::LapPoint::Response &res)
    {
        ROS_WARN("callback started for State Estimation map");
        res.way_point_origin = impl_->gps_origin_point;
        res.lap_check_in = impl_->lap_check_start_point;
        res.lap_check_out = impl_->lap_check_end_point;
        return true;
    }

    void DataResourceManager::CalculateCentroidOfTrackIn()
    {
        impl_->gps_origin_point.lon = 0;
        impl_->gps_origin_point.lat = 0;
        int i = 0;

        for (auto &p : impl_->readed_highway_in)
        {
            impl_->gps_origin_point.lon += p.lon;
            impl_->gps_origin_point.lat += p.lat;
            i++;
        }

        impl_->gps_origin_point.lon /= i;
        impl_->gps_origin_point.lat /= i;

        ROS_WARN("center of in lat: %f", impl_->gps_origin_point.lat);
        ROS_WARN("center of in lon: %f", impl_->gps_origin_point.lon);
    }

    ///
    //! @brief Read text files and save in vectors. Not a memeber function
    ///
    void ReadPoints(const std::string &path_src, std::vector<rr_common::PointGps> &dst)
    {
        std::ifstream istr;
        double lon, lat;
        rr_common::PointGps temp;
        int size = 0;

        istr.open(path_src, std::ios::in);

        if (!istr.is_open())
        {
            ROS_FATAL("Cannot open file: %s", path_src.c_str());
        }

        dst.clear();

        while (istr >> lat && istr >> lon)
        {
            temp.lat = lat;
            temp.lon = lon;
            dst.push_back(temp);
            size++;
        }

        istr.close();

        ROS_WARN("read %d points from %s", size, path_src.c_str());
    }
}
