#include "rr_path_planner/path_generator.h"

namespace rr_path_planner
{
    struct PathGenerator::Impl
    {
        // local path in vector<PointXY>
        std::vector<rr_common::PointXY> path_xy;

        // map in vector<float>
        std::vector<rr_common::PointXY> map_in;
        std::vector<rr_common::PointXY> map_out;
        // map in vector<PointY>
        std::vector<rr_common::PointXY> map_in_xy;
        std::vector<rr_common::PointXY> map_out_xy;

        // size of coure 1 and 2
        int in_size;
        int out_size;

        // current position in PointY
        rr_common::PointXY position_xy;

        // generated local path in vector<PointXY>
        std::vector<rr_common::PointXY> local_path_xy;

        // control points for bezier curve
        rr_common::PointXY P0, P1, P2, P3;

        // bezier curve points splitted in 0.5m
        std::vector<rr_common::PointXY> splitted_bezier_points;

        // buffer size of local path
        int buffer_size;

        // course from strategy
        int course;

        // value to know is robot in curve or not
        bool change;
    };

    PathGenerator::PathGenerator() : impl_(new Impl) {}
    PathGenerator::~PathGenerator() {}

    void PathGenerator::Init(ros::NodeHandle &nh,
                             const std::string &buffer_size)
    {
        ROS_ASSERT(nh.getParam(buffer_size, impl_->buffer_size));
    }

    std::vector<rr_common::PointXY> PathGenerator::GetPath()
    {
        return impl_->local_path_xy;
    }

    void PathGenerator::ClearPath()
    {
        // clear all points
        impl_->local_path_xy.clear();
        impl_->splitted_bezier_points.clear();
        impl_->change = false;
    }

    void PathGenerator::SetCourse(int course)
    {
        impl_->course = course;
    }

    void PathGenerator::SetState(const rr_common::PointXY &state)
    {
        impl_->position_xy = state;
    }

    void PathGenerator::SetMap(const std::vector<rr_common::PointXY> &map_in,
                               const std::vector<rr_common::PointXY> &map_out)
    {
        impl_->map_in = map_in;
        impl_->map_out = map_out;

        impl_->in_size = impl_->map_in_xy.size();
        impl_->out_size = impl_->map_out_xy.size();
    }

    void PathGenerator::CreateLocalPath()
    {
        impl_->local_path_xy.clear();
        if (impl_->course < 10)
        {
            StraightPath();
        }
        else
        {
            if (!impl_->change)
            {
                // calculate Bezier curve once
                impl_->splitted_bezier_points.clear();
                FindControlPoints();
                MakeBezierCurve();
            }
            // make course changing path
            ChangingPath();
        }
    }

    void PathGenerator::StraightPath()
    {
        // make path containing buffer size elements from current position
        // simply add points of current course

        if (impl_->course == 1)
        {
            int path_start_idx = FindClosestPointPath(impl_->map_in_xy, impl_->position_xy).idx;
            for (int i = 0; i < impl_->buffer_size; i++)
            {
                impl_->local_path_xy.push_back(impl_->map_in_xy[(path_start_idx + i) % impl_->in_size]);
            }
        }

        if (impl_->course == 2)
        {
            int path_start_idx = FindClosestPointPath(impl_->map_out_xy, impl_->position_xy).idx;
            for (int i = 0; i < impl_->buffer_size; i++)
            {
                impl_->local_path_xy.push_back(impl_->map_out_xy[(path_start_idx + i) % impl_->out_size]);
            }
        }
    }

    void PathGenerator::FindControlPoints()
    {
        // 1->2
        if (impl_->course == 12)
        {
            impl_->P0 = FindClosestPointPath(impl_->map_in_xy, impl_->position_xy);
            impl_->P1 = impl_->map_in_xy[(impl_->P0.idx + 5) % impl_->in_size];
            impl_->P2 = impl_->map_out_xy
                            [(FindClosestPointPath(impl_->map_out_xy, impl_->P1).idx + 20) %
                             impl_->out_size];
            impl_->P3 = impl_->map_out_xy[(impl_->P2.idx + 5) % impl_->out_size];
        }

        // 2->1
        if (impl_->course == 21)
        {
            impl_->P0 = FindClosestPointPath(impl_->map_out_xy, impl_->position_xy);
            impl_->P1 = impl_->map_out_xy[(impl_->P0.idx + 5) % impl_->out_size];
            impl_->P2 = impl_->map_in_xy
                            [(FindClosestPointPath(impl_->map_in_xy, impl_->P1).idx + 20) %
                             impl_->in_size];
            impl_->P3 = impl_->map_in_xy[(impl_->P2.idx + 5) % impl_->in_size];
        }
    }

    void PathGenerator::MakeBezierCurve()
    {
        rr_common::PointXY B;
        rr_common::PointXY prev = impl_->P0;
        int delta_size = 150;
        float length = 0;
        std::vector<rr_common::PointXY> bezier_points;

        ///
        //!@brief split curve in 150 parts
        //! assume sum of the distance between two point is length of curve
        ///
        for (int i = 0; i < delta_size; i++)
        {
            float t = float(i) / delta_size;
            B.x = impl_->P0.x * pow(1 - t, 3) +
                  3 * impl_->P1.x * t * pow(1 - t, 2) +
                  3 * impl_->P2.x * (1 - t) * pow(t, 2) +
                  impl_->P3.x * pow(t, 3);
            B.y = impl_->P0.y * pow(1 - t, 3) +
                  3 * impl_->P1.y * t * pow(1 - t, 2) +
                  3 * impl_->P2.y * (1 - t) * pow(t, 2) +
                  impl_->P3.y * pow(t, 3);
            bezier_points.push_back(B);
            length += CalDistancePath(B, prev);
            prev = B;
        }

        ///
        //!@brief search the point which distance is about 0.5m from previous point
        ///
        float dis = 0;
        prev = bezier_points[0];
        int idx = 0;
        for (auto &point : bezier_points)
        {
            dis = CalDistancePath(point, prev);

            if (0.4 < dis)
            {
                point.idx = idx;
                impl_->splitted_bezier_points.push_back(point);
                dis = 0;
                if (idx == 29)
                {
                    break;
                }
                idx++;
                prev = point;
            }
        }

        impl_->change = true;
    }

    void PathGenerator::ChangingPath()
    {
        // impl_->local_path_xy.clear();

        rr_common::PointXY cur = FindClosestPointPath(impl_->splitted_bezier_points, impl_->position_xy);

        ///
        //!@brief maintain curve path and add path at the end of curve
        //!

        // size of splitted bezier curve
        int curve_size = impl_->splitted_bezier_points.size();

        // if robot is still on curve
        if (cur.idx < curve_size - 1)
        {
            for (int i = 0; i < impl_->buffer_size; i++)
            {
                if (i < curve_size - cur.idx)
                {
                    impl_->local_path_xy.push_back(
                        impl_->splitted_bezier_points[cur.idx + i]);
                }
                else
                {
                    if (impl_->course == 12)
                    {
                        impl_->local_path_xy.push_back(
                            impl_->map_out_xy[impl_->P3.idx - curve_size + cur.idx + i - 1]);
                    }
                    if (impl_->course == 21)
                    {
                        impl_->local_path_xy.push_back(
                            impl_->map_in_xy[impl_->P3.idx - curve_size + cur.idx + i - 1]);
                    }
                }
            }
        }
        // if robot finish lane change and on straight path
        else
        {
            impl_->splitted_bezier_points.clear();
            impl_->change = false;
        }
    }

    float PathGenerator::CalDistancePath(rr_common::PointXY point1, rr_common::PointXY point2)
    {
        return sqrt(pow((point1.x - point2.x), 2) + pow((point1.y - point2.y), 2));
    }

    rr_common::PointXY PathGenerator::FindClosestPointPath(const std::vector<rr_common::PointXY> &points, rr_common::PointXY src)
    {
        rr_common::PointXY closest;
        float distance = CalDistancePath(src, points[0]);
        for (auto &temp : points)
        {
            if (CalDistancePath(src, temp) <= distance)
            {
                distance = CalDistancePath(src, temp);
                closest.x = temp.x;
                closest.y = temp.y;
                closest.idx = temp.idx % points.size();
            }
        }

        return closest;
    }
}