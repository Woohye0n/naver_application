#include "rr_path_planner/on_course_checker.h"

namespace rr_path_planner
{
    struct OnCourseChecker::Impl
    {
        // PointXY vector of local path
        std::vector<rr_common::PointXY> path_xy;

        // PointXY point of current position
        rr_common::PointXY position_xy;

        // allowed distance of current position and path
        float path_diff_allowed;
    };

    OnCourseChecker::OnCourseChecker() : impl_(new Impl) {}
    OnCourseChecker::~OnCourseChecker() {}

    void OnCourseChecker::Init(ros::NodeHandle &nh,
                               const std::string &path_diff_allowed)
    {
        ROS_ASSERT(nh.getParam(path_diff_allowed, impl_->path_diff_allowed));
    }

    bool OnCourseChecker::IsOk()
    {
        // calculate minimum distance to path
        float dis_to_path = FindMinDistanceOn(impl_->path_xy, impl_->position_xy);

        return dis_to_path < impl_->path_diff_allowed;
    }

    void OnCourseChecker::SetPath(const std::vector<rr_common::PointXY> &path)
    {
        // clear before push back to vector
        impl_->path_xy.clear();

        // update path
        impl_->path_xy = path;
    }

    void OnCourseChecker::SetState(const rr_common::PointXY &state)
    {
        impl_->position_xy = state;
    }

    float OnCourseChecker::CalDistanceOn(rr_common::PointXY point1, rr_common::PointXY point2)
    {
        return sqrt(pow((point1.x - point2.x), 2) + pow((point1.y - point2.y), 2));
    }

    float OnCourseChecker::FindMinDistanceOn(const std::vector<rr_common::PointXY> &points, rr_common::PointXY src)
    {
        float distance = CalDistanceOn(src, points[0]);
        for (auto &temp : points)
        {
            if (CalDistanceOn(src, temp) <= distance)
            {
                distance = CalDistanceOn(src, temp);
            }
        }
        return distance;
    }
} // namespace rr_path_planner