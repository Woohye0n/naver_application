#ifndef RR_PERCEPTION_RR_OBJECT_DETECTION_RANSAC_PLANE_H_
#define RR_PERCEPTION_RR_OBJECT_DETECTION_RANSAC_PLANE_H_

#include "rr_object_detection/index_filter.h"
#include <random>

namespace rr_perception
{
    using PointT = pcl::PointXYZ;
    class RANSACPlane
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        RANSACPlane() : count_max_(0){};
        ~RANSACPlane() {}
        void Compute(pcl::PointCloud<PointT>::Ptr &points, int num_sample, float tolerance)
        {
            PlaneIndexFilter<PointT> filter;
            std::uniform_int_distribution<> idx_gen(0, points->size() - 1);

            for (int i = 0; i < num_sample; ++i)
            {
                // create candidate
                Eigen::Vector3f pivot = points->at(idx_gen(reng_)).getVector3fMap();
                Eigen::Vector3f a = points->at(idx_gen(reng_)).getVector3fMap();
                Eigen::Vector3f b = points->at(idx_gen(reng_)).getVector3fMap();
                Eigen::Vector3f new_n = (a - pivot).cross(b - pivot);
                new_n.normalize();
                float new_d = -new_n.dot(pivot);
                // a,b,c,d representation is redundant for plane fitting. only 3 DoF for the planes with unsigned normal.
                // make c > 0 as constraint.
                if (new_n(2) < 0)
                {
                    new_n = -1.0 * new_n;
                    new_d = -1.0 * new_d;
                }

                pcl::PointIndices::Ptr idx(new pcl::PointIndices());
                filter.SetPlane(new_n, new_d, tolerance, pcl::ComparisonOps::EQ);
                filter.Filter(points, idx);
                if (idx->indices.size() > count_max_)
                {
                    count_max_ = idx->indices.size();
                    n_ = new_n;
                    d_ = new_d;
                    inliers_.swap(idx);
                }
            }
        }
        Eigen::Vector3f GetPlaneNormal()
        {
            return n_;
        }
        float GetPlaneD()
        {
            return d_;
        }
        pcl::PointIndices::Ptr GetInlierIndices()
        {
            return inliers_;
        }

    private:
        Eigen::Vector3f n_;
        float d_;
        int count_max_;
        pcl::PointIndices::Ptr inliers_;
        std::default_random_engine reng_;
    };

}

#endif //RR_PERCEPTION_RR_OBJECT_DETECTION_RANSAC_PLANE_H_