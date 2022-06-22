#include "rr_path_planner/speed_generator.h"

namespace rr_path_planner
{
    struct SpeedGenerator::Impl
    {
        std::vector<rr_common::PointXY> path_xy;
        rr_common::PointXY position;
        float speed = 0;
        float curvature = 0;
        float min_radius;
        float min_vel;
    };

    SpeedGenerator::SpeedGenerator() : impl_(new Impl) {}
    SpeedGenerator::~SpeedGenerator() {}

    void SpeedGenerator::Init(ros::NodeHandle &nh)
    {
        ROS_ASSERT(nh.getParam("min_radius", impl_->min_radius));
        ROS_ASSERT(nh.getParam("min_vel", impl_->min_vel));
    }

    void SpeedGenerator::SetPath(std::vector<rr_common::PointXY> &path)
    {
        // clear before push back to vector
        impl_->path_xy.clear();
        impl_->path_xy = path;
    }

    void SpeedGenerator::SetState(const rr_common::PointXY &state)
    {
        impl_->position = state;
    }

    float SpeedGenerator::GetSpeed()
    {
        return impl_->speed;
    }

    void SpeedGenerator::MakeVector()
    {
        if (impl_->path_xy.size() == 30)
        {
            int start_point = (FindClosestPointSpeed(impl_->path_xy, impl_->position).idx) %
                              impl_->path_xy.size();

            Eigen::Vector2f p0;
            Eigen::Vector2f p1;
            Eigen::Vector2f p2;
            p0 << impl_->path_xy[start_point % impl_->path_xy.size()].x,
                impl_->path_xy[start_point % impl_->path_xy.size()].y;
            p1 << impl_->path_xy[(start_point + 14) % impl_->path_xy.size()].x,
                impl_->path_xy[(start_point + 14) % impl_->path_xy.size()].y;
            p2 << impl_->path_xy[(start_point + 29) % impl_->path_xy.size()].x,
                impl_->path_xy[(start_point + 29) % impl_->path_xy.size()].y;

            impl_->curvature = CalCurvature(p0, p1, p2);

            // velocity curve
            if (impl_->curvature < (1 / impl_->min_radius))
            {
                impl_->speed = 20;
            }
            else
            {
                impl_->speed = 20 + ((20 - impl_->min_vel) / (1 / impl_->min_radius - 1 / 3.5)) *
                                        (impl_->curvature - 1 / impl_->min_radius);
                if (impl_->speed < 0)
                {
                    impl_->speed = 0;
                }
                if (impl_->speed > 20)
                {
                    impl_->speed = 20;
                }
            }
        }
    }

    float SpeedGenerator::CalCurvature(const Eigen::Vector2f &p0,
                                       const Eigen::Vector2f &p1,
                                       const Eigen::Vector2f &p2)
    {
        ///
        //! @brief Get the Curvature of the point p_cur.
        //! Use circumscribed circle approximation.
        //! if all points are on the same line, curvature is 0
        //!
        //! @param[in] p0 p0 on the path. first point of the path. vehicle is between p0 and p1
        //! @param[in] p1 p1 on the path. p1 is next point of the p0.
        //! @param[in] p2 p2 on the path. p2 is next point of the p1
        ///
        const Eigen::Vector2f &p10 = p1 - p0;
        const Eigen::Vector2f &p12 = p1 - p2;
        Eigen::Matrix2f A;
        A << p10.transpose(), p12.transpose();
        if (abs(A.determinant()) < 0.000000000001)
        {
            return 0.0;
        }
        Eigen::Vector2f b;
        b << p0.dot(p10), p2.dot(p12);
        Eigen::Vector2f c = A.inverse() * b;
        return 1.0 / (c - p0).norm();
    }

    float SpeedGenerator::CalDistanceSpeed(rr_common::PointXY point1, rr_common::PointXY point2)
    {
        return sqrt(pow((point1.x - point2.x), 2) + pow((point1.y - point2.y), 2));
    }

    rr_common::PointXY SpeedGenerator::FindClosestPointSpeed(const std::vector<rr_common::PointXY> &points, rr_common::PointXY src)
    {
        rr_common::PointXY closest;
        float distance = CalDistanceSpeed(src, points[0]);
        for (auto &temp : points)
        {
            if (CalDistanceSpeed(src, temp) <= distance)
            {
                distance = CalDistanceSpeed(src, temp);
                closest = temp;
                closest.idx = temp.idx % points.size();
            }
        }

        return closest;
    }

}