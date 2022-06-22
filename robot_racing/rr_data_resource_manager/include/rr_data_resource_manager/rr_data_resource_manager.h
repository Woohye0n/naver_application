#ifndef RR_DATA_RESOURCE_MANAGER_DATA_RESOURCE_MANAGER_RR_DATA_RESOURCE_MANAGER_H_
#define RR_DATA_RESOURCE_MANAGER_DATA_RESOURCE_MANAGER_RR_DATA_RESOURCE_MANAGER_H_

#include "rr_common/GpsTrackData.h"
#include "rr_common/LapPoint.h"
#include "rr_common/PointGps.h"

#include <ros/ros.h>
#include <vector>
#include <iostream>

namespace rr_data_resource_manager
{
    class DataResourceManager
    {
    public:
        void Init(ros::NodeHandle &nh);

        DataResourceManager();
        ~DataResourceManager();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;

        bool AddMapSrv(rr_common::GpsTrackData::Request &req, rr_common::GpsTrackData::Response &res);
        bool AddSESrv(rr_common::LapPoint::Request &req, rr_common::LapPoint::Response &res);
        void CalculateCentroidOfTrackIn();
    };

    void ReadPoints(const std::string &path_src, std::vector<rr_common::PointGps> &dst);
}

#endif // RR_DATA_RESOURCE_MANAGER_DATA_RESOURCE_MANAGER_RR_DATA_RESOURCE_MANAGER_H_