#ifndef RR_MAP_RR_MAP_RR_MAP_H_
#define RR_MAP_RR_MAP_RR_MAP_H_

#include "rr_common/GpsTrackData.h"
#include "rr_common/MapWayPointXY.h"
#include "rr_common/PointXY.h"
#include "rr_common/PointGps.h"

#include <ros/ros.h>
#include <vector>
#include <iostream>

namespace rr_map
{
    class Map
    {
    public:
        void Init(ros::NodeHandle &nh);
        Map();
        ~Map();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;

        // function
        bool AddMapXYSrv(rr_common::MapWayPointXY::Request &req, rr_common::MapWayPointXY::Response &res);
        void SaveVector(std::vector<rr_common::PointGps> &gps, std::vector<rr_common::PointXY> &dst);
        rr_common::PointXY ConvertGps2XY(rr_common::PointGps p);
    };
}
#endif