#ifndef RR_PERCEPTION_RR_OBJECT_DETECTION_OBSTACLE_CLUSTER_EXTRACTOR_H_
#define RR_PERCEPTION_RR_OBJECT_DETECTION_OBSTACLE_CLUSTER_EXTRACTOR_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <memory>

namespace rr_perception
{
    class ObstacleClusterExtractor
    {
    public:
        ObstacleClusterExtractor();
        ~ObstacleClusterExtractor();
        void Init(int min_cluster_size,
                  int max_cluster_size,
                  float max_neighbor_distance);
        void Extract(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in,
                     pcl::PointCloud<pcl::PointXYZL>::Ptr pc_label_out,
                     std::vector<pcl::PointIndices> &indicies_out);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

};

#endif // RR_PERCEPTION_RR_OBJECT_DETECTION_OBSTACLE_CLUSTER_EXTRACTOR_H_