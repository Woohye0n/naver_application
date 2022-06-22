#ifndef RR_PATH_PLANNER_RR_PATH_PLANNER_PATH_GENERATOR_H_
#define RR_PATH_PLANNER_RR_PATH_PLANNER_PATH_GENERATOR_H_

#include "rr_common/PointXY.h"

#include <ros/ros.h>
#include <vector>
#include <memory>

namespace rr_path_planner
{
    class PathGenerator
    {
    public:
        PathGenerator();
        ~PathGenerator();
        void Init(ros::NodeHandle &nh,
                  const std::string &buffer_size);
        bool IsOk();
        void SetCourse(int course);
        void SetMap(const std::vector<rr_common::PointXY> &map_in,
                    const std::vector<rr_common::PointXY> &map_out);
        void SetState(const rr_common::PointXY &state);

        std::vector<rr_common::PointXY> GetPath();
        void ClearPath();
        void CreateLocalPath();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;

        void StraightPath();
        void FindControlPoints();
        void MakeBezierCurve();
        void ChangingPath();
        float CalDistancePath(rr_common::PointXY point1, rr_common::PointXY point2);
        rr_common::PointXY FindClosestPointPath(const std::vector<rr_common::PointXY> &points, rr_common::PointXY src);
    };

} // namespace rr_path_planner

#endif // RR_PATH_PLANNER_RR_PATH_PLANNER_PATH_GENERATOR_H_