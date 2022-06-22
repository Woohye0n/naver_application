#include "rr_object_tracker/box_types.h"
#include "rr_object_tracker/polygon.h"

namespace rr_perception
{
    float GetArea(const OBB &b) { return (b.p0 - b.p_pivot).norm() * (b.p1 - b.p_pivot).norm(); }
    float GetRadius(const OBB &b) { return (b.p0 - b.p1).norm() * 0.5; }
    Eigen::Vector2f GetCenter(const OBB &b) { return (b.p0 + b.p1) * 0.5; }
    Eigen::Vector2f GetP2(const OBB &b) { return b.p0 + b.p1 - b.p_pivot; }
    float GetCenterDistance(const OBB &b0, const OBB &b1) { return (GetCenter(b0) - GetCenter(b1)).norm(); }
    float GetIoU(const OBB &b0, const OBB &b1)
    {
        // prune
        if (GetRadius(b0) + GetRadius(b1) < GetCenterDistance(b0, b1))
        {
            return 0.0;
        }

        // cycle order : 0, p, 1, 2
        Polygon p;
        p.push_back(b0.p0);
        p.push_back(b0.p_pivot);
        p.push_back(b0.p1);
        p.push_back(GetP2(b0));

        Polygon q;
        q.push_back(b1.p0);
        q.push_back(b1.p_pivot);
        q.push_back(b1.p1);
        q.push_back(GetP2(b1));

        Polygon inter;
        Intersection(p, q, inter);
        // make IoU
        const float &intersection_area = Area(inter);
        return intersection_area / (GetArea(b0) + GetArea(b1) - intersection_area);
    }

    // row: test, col: gt
    Eigen::MatrixXf IoUBatch(std::vector<OBB> &test, std::vector<OBB> &gt)
    {
        Eigen::MatrixXf sim;
        sim.resize(test.size(), gt.size());
        for (size_t r = 0; r < test.size(); ++r)
        {
            for (size_t c = 0; c < gt.size(); ++c)
            {
                sim(r, c) = GetIoU(test[r], gt[c]);
            }
        }
        return sim;
    }

    Eigen::MatrixXf DistanceBatch(std::vector<OBB> &test, std::vector<OBB> &gt)
    {
        Eigen::MatrixXf sim;
        sim.resize(test.size(), gt.size());
        for (size_t r = 0; r < test.size(); ++r)
        {
            for (size_t c = 0; c < gt.size(); ++c)
            {
                sim(r, c) = GetCenterDistance(test[r], gt[c]);
            }
        }
        return sim;
    }

    float GetIoU(const AABB &test, const AABB &gt)
    {
        const float &x1 = test.x1 > gt.x1 ? test.x1 : gt.x1;
        const float &y1 = test.y1 > gt.y1 ? test.y1 : gt.y1;
        const float &x2 = test.x2 < gt.x2 ? test.x2 : gt.x2;
        const float &y2 = test.y2 < gt.y2 ? test.y2 : gt.y2;
        const float &w = 0.0 > x2 - x1 ? 0.0 : x2 - x1;
        const float &h = 0.0 > y2 - y1 ? 0.0 : y2 - y1;
        const float &inter = w * h;
        const float &a_test = (test.x2 - test.x1) * (test.y2 - test.y1);
        const float &a_gt = (gt.x2 - gt.x1) * (gt.y2 - gt.y1);
        const float &uni = a_test + a_gt - inter;
        return inter / uni;
    }

    // row: test, col: gt
    Eigen::MatrixXf IoUBatch(std::vector<AABB> &test, std::vector<AABB> &gt)
    {
        Eigen::MatrixXf sim;
        sim.resize(test.size(), gt.size());
        for (size_t r = 0; r < test.size(); ++r)
        {
            for (size_t c = 0; c < gt.size(); ++c)
            {
                sim(r, c) = GetIoU(test[r], gt[c]);
            }
        }
        return sim;
    }
}
