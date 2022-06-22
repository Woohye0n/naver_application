#include "rr_object_detection/obstacle_cluster_extractor.h"
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace rr_perception
{
    struct ObstacleClusterExtractor::Impl
    {
        int min_cluster_size;
        int max_cluster_size;
        float max_neighbor_distance;

        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster;
    };
    ObstacleClusterExtractor::ObstacleClusterExtractor() : impl_(new Impl)
    {
    }
    ObstacleClusterExtractor::~ObstacleClusterExtractor()
    {
    }
    void ObstacleClusterExtractor::Init(int min_cluster_size,
                                        int max_cluster_size,
                                        float max_neighbor_distance)
    {
        impl_->min_cluster_size = min_cluster_size;
        impl_->max_cluster_size = max_cluster_size;
        impl_->max_neighbor_distance = max_neighbor_distance;
        impl_->kdtree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
    }
    void ObstacleClusterExtractor::Extract(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in,
                                           pcl::PointCloud<pcl::PointXYZL>::Ptr pc_label_out,
                                           std::vector<pcl::PointIndices> &indicies_out)
    {
        // indicies_out.clear();
        if (pc_in->size() == 0)
        {
            return;
        }
        impl_->kdtree->setInputCloud(pc_in);
        impl_->cluster.setClusterTolerance(impl_->max_neighbor_distance);
        impl_->cluster.setMinClusterSize(impl_->min_cluster_size);
        impl_->cluster.setMaxClusterSize(impl_->max_cluster_size); // no limit.
        impl_->cluster.setSearchMethod(impl_->kdtree);
        impl_->cluster.setInputCloud(pc_in);
        impl_->cluster.extract(indicies_out);

        int label = 0;
        for (auto &idcs : indicies_out)
        {
            for (auto &idx : idcs.indices)
            {
                pc_label_out->at(idx).label = label;
            }
            label++;
        }
    }
}
