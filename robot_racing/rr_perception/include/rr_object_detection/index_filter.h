#ifndef RR_PERCEPTION_RR_OBJECT_DETECTION_INDEX_FILTER_H_
#define RR_PERCEPTION_RR_OBJECT_DETECTION_INDEX_FILTER_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/conditional_removal.h>
#include <velodyne_pointcloud/point_types.h>
#include <iostream>

namespace rr_perception
{

    template <typename PointT>
    class IndexFilter
    {

    public:
        IndexFilter(){};
        ~IndexFilter(){};

        void Filter(
            const boost::shared_ptr<const pcl::PointCloud<PointT>> &input,
            pcl::PointIndices::Ptr &indicies)

        {
            auto iter = input->begin();
            auto end = input->end();
            int i = 0;
            while (iter != end)
            {
                if (Evaluate(*iter))
                {
                    indicies->indices.push_back(i);
                }
                i++;
                iter++;
            }
        }

    protected:
        virtual bool Evaluate(const PointT &point) = 0;
    };

    class RingIndexFilter : public IndexFilter<velodyne_pointcloud::PointXYZIR>
    {
    public:
        RingIndexFilter(int ring_idx) : ring_idx_(ring_idx) {}
        ~RingIndexFilter() = default;
        virtual bool Evaluate(const velodyne_pointcloud::PointXYZIR &point)
        {
            return point.ring == ring_idx_;
        }

    private:
        int ring_idx_;
    };

    template <typename PointT>
    class PlaneIndexFilter : public IndexFilter<PointT>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        PlaneIndexFilter() = default;
        ~PlaneIndexFilter() = default;
        void SetPlane(const Eigen::Vector3f &abc, float d, float tolerance, pcl::ComparisonOps::CompareOp op)
        {
            // normalize n_ and d_
            const float &norm = abc.norm();
            n_ = abc / norm;
            d_ = d / norm;
            tolerance_ = tolerance;
            op_ = op;
        }
        virtual bool Evaluate(const PointT &point)
        {
            Eigen::Vector3f p = point.getVector3fMap();
            const float &err = (n_.dot(p) + d_);

            switch (op_)
            {
            case pcl::ComparisonOps::GT:
                return err > tolerance_;
            case pcl::ComparisonOps::GE:
                return err >= -tolerance_;
            case pcl::ComparisonOps::LT:
                return err < -tolerance_;
            case pcl::ComparisonOps::LE:
                return err <= tolerance_;
            case pcl::ComparisonOps::EQ:
                return err <= tolerance_ && err >= -tolerance_;
            default:
                return false;
            }
            return err < tolerance_;
        }

    private:
        pcl::ComparisonOps::CompareOp op_;
        Eigen::Vector3f n_;
        float d_;
        float tolerance_;
    };
}

#endif //RR_PERCEPTION_RR_OBJECT_DETECTION_INDEX_FILTER_H_