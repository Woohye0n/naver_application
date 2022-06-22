#ifndef RR_PERCEPTION_RR_OBJECT_TRACKER_POLYGON_H_
#define RR_PERCEPTION_RR_OBJECT_TRACKER_POLYGON_H_

#include <Eigen/Dense>
#include <vector>
#include <algorithm>

namespace rr_perception
{
    using Polygon = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;

    inline float Area(const Polygon &poly)
    {
        float area = 0.0;
        for (auto it = poly.begin(); it != poly.end(); it++)
        {
            // const Eigen::Vector3f p0((*it)(0), (*it)(1), 0.0);
            // const Eigen::Vector2f &p1 = (it + 1) == poly.end() ? poly.front() : *(it + 1);
            // const Eigen::Vector3f p1_3d(p1(0), p1(1), 0.0);

            const Eigen::Vector2f &p0 = *it;
            const Eigen::Vector2f &p1 = (it + 1) == poly.end() ? poly.front() : *(it + 1);

            area += (p0(0) * p1(1) - p0(1) * p1(0));
        }
        return 0.5 * fabs(area);
    }

    inline bool Intersection(const Polygon &poly0, const Polygon &poly1, Polygon &contour)
    {
        // copy vector
        contour = poly0;

        // iterate over cutting edges
        for (auto q0_it = poly1.begin(); q0_it != poly1.end(); q0_it++)
        {
            // select points from q
            const Eigen::Vector2f &q0 = *q0_it;
            const Eigen::Vector2f &q1 = (q0_it + 1) == poly1.end() ? poly1.front() : *(q0_it + 1);
            const Eigen::Vector2f &q2 = (q0_it) == poly1.begin() ? poly1.back() : *(q0_it - 1);
            const Eigen::Vector3f u1(q1(0) - q0(0), q1(1) - q0(1), 0.0);
            const Eigen::Vector3f u2(q2(0) - q0(0), q2(1) - q0(1), 0.0);

            const Eigen::Vector2f &n = u1.cross(u2).cross(u1).block(0, 0, 2, 1);

            // cutting line is q0 + t*(q1) with param t.

            // iterate over polygon p and reorder.
            auto p0_it = contour.begin();
            for (; p0_it != contour.end() && ((*p0_it - q0).transpose() * n) < 0; p0_it++)
            {
            }

            // no intersection
            if (p0_it == contour.end())
            {
                return false;
            }

            // reorder polygon
            std::rotate(contour.begin(), p0_it, contour.end());
            // Polygon::const_iterator end = p0_it;
            // contour.reserve(contour.size());
            // do
            // {
            //     contour.push_back(*p0_it);
            //     p0_it++;
            //     if (p0_it == contour.end())
            //     {
            //         p0_it = contour.begin();
            //     }
            // } while (p0_it != end);

            // reduce the polygon
            Polygon::iterator c_it = contour.begin();
            for (; c_it != contour.end();)
            {
                const Eigen::Vector2f &p0 = *c_it;
                const Eigen::Vector2f &p1 = (c_it + 1) == contour.end() ? contour.front() : *(c_it + 1);
                const float &s0 = (p0 - q0).transpose() * n;
                const float &s1 = (p1 - q0).transpose() * n;

                if (s0 > 0)
                {
                    if (s1 < 0)
                    {
                        float k = -s0 / ((p1 - p0).transpose() * n);
                        c_it = contour.insert(c_it + 1, p0 + k * (p1 - p0));
                    }
                    c_it++;
                }
                else if (s0 == 0)
                {
                    if (s1 == 0)
                    {
                        c_it = contour.erase(c_it);
                    }
                    else
                    {
                        c_it++;
                    }
                }
                else if (s0 < 0)
                {
                    if (s1 > 0)
                    {
                        float k = -s0 / ((p1 - p0).transpose() * n);
                        *c_it = p0 + k * (p1 - p0);
                        c_it++;
                    }
                    else
                    {
                        c_it = contour.erase(c_it);
                    }
                }
            }
        }
        return true;
    }

} // namespace rr_perception

#endif // RR_PERCEPTION_RR_OBJECT_TRACKER_POLYGON_H_