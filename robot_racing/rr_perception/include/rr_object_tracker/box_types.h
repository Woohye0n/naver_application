#ifndef RR_PERCEPTION_RR_OBJECT_TRACKER_BOX_TYPES_H_
#define RR_PERCEPTION_RR_OBJECT_TRACKER_BOX_TYPES_H_

#include <Eigen/Dense>
#include <vector>

namespace rr_perception
{
    struct OBB
    {
        // point1 is next to the pivot.
        // point2 is next to the pivot, but oposite side to the point1.
        // 4 points would be point1, point2, point_pivot, (point1 + point2 - point_pivot)
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector2f p0;
        Eigen::Vector2f p1;
        Eigen::Vector2f p_pivot;
    };

    float GetArea(const OBB &b);
    float GetRadius(const OBB &b);
    Eigen::Vector2f GetCenter(const OBB &b);
    Eigen::Vector2f GetP2(const OBB &b);
    float GetCenterDistance(const OBB &b0, const OBB &b1);
    float GetIoU(const OBB &b0, const OBB &b1);
    Eigen::MatrixXf IoUBatch(std::vector<OBB> &test, std::vector<OBB> &gt);
    Eigen::MatrixXf DistanceBatch(std::vector<OBB> &test, std::vector<OBB> &gt);

    struct AABB
    {
        float x1; // min x
        float y1; // min y
        float x2; // max x
        float y2; // max y
    };

    float GetIoU(const AABB &test, const AABB &gt);
    Eigen::MatrixXf IoUBatch(std::vector<AABB> &test, std::vector<AABB> &gt);
}

#endif //RR_PERCEPTION_RR_OBJECT_TRACKER_BOX_TYPES_H_