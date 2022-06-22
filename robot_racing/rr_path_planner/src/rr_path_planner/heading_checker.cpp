#include "rr_path_planner/heading_checker.h"

namespace rr_path_planner
{
    struct HeadingChecker::Impl
    {
        std::vector<rr_common::PointXY> path_xy;

        rr_common::PointXY position_xy;

        rr_common::PointXY WL;

        // current heading
        float heading;

        // degree difference between heading and heading of atan2(path[0]. path[1])
        float deg_diff;

        // degree difference allowed to follow path
        float deg_diff_allowed;
    };

    HeadingChecker::HeadingChecker() : impl_(new Impl) {}
    HeadingChecker::~HeadingChecker() {}

    void HeadingChecker::Init(ros::NodeHandle &nh,
                              const std::string &deg_diff_allowed)
    {
        ROS_ASSERT(nh.getParam(deg_diff_allowed, impl_->deg_diff_allowed));

        // initilaize heading
        impl_->heading = 0;
    }

    bool HeadingChecker::IsOk()
    {
        FindWL();
        const float &X = impl_->WL.x - impl_->position_xy.x;
        const float &Y = impl_->WL.y - impl_->position_xy.y;
        const float &head_to_path = fabs(atan2(X, Y) * (180 / M_PI) - impl_->heading);

        return (head_to_path < 45) ? true : false;
    }

    void HeadingChecker::SetPath(const std::vector<rr_common::PointXY> &path)
    {
        // clear before push back to vector
        impl_->path_xy.clear();

        // update path
        impl_->path_xy = path;
    }

    void HeadingChecker::SetHeading(const float &heading)
    {
        impl_->heading = heading;
    }

    void HeadingChecker::SetState(rr_common::PointXY &state)
    {
        impl_->position_xy.x = state.x;
        impl_->position_xy.y = state.y;
    }

    void HeadingChecker::FindWL()
    {
        impl_->WL = impl_->path_xy[6];
    }

    float HeadingChecker::CalDistanceHead(rr_common::PointXY point1, rr_common::PointXY point2)
    {
        return sqrt(pow((point1.x - point2.x), 2) + pow((point1.y - point2.y), 2));
    }

    float HeadingChecker::FindMinDistanceHead(const std::vector<rr_common::PointXY> &points, rr_common::PointXY src)
    {
        float distance = CalDistanceHead(src, points[0]);
        for (auto &temp : points)
        {
            if (CalDistanceHead(src, temp) <= distance)
            {
                distance = CalDistanceHead(src, temp);
            }
        }
        return distance;
    }
}
