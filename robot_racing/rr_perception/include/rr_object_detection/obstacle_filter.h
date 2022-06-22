#ifndef RR_PERCEPTION_RR_OBJECT_DETECTION_OBSTACLE_FILTER_H_
#define RR_PERCEPTION_RR_OBJECT_DETECTION_OBSTACLE_FILTER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace rr_perception
{

    class ObstacleFilter
    {
    public:
        ObstacleFilter();
        ~ObstacleFilter();
        void Init(float lidar_height,
                  float seg_deg,
                  int bin_num,
                  float max_dis,
                  float T_m,
                  float T_b,
                  float T_th,
                  float T_d);
        void Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_prj,
                    pcl::PointCloud<pcl::PointXYZL>::Ptr pc_pos_out);
        void Segmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc_in,
                          std::vector<pcl::PointCloud<pcl::PointXYZ>> *segments);
        void BinMapping(const std::vector<pcl::PointCloud<pcl::PointXYZ>> *segments);
        void ExtractLine(const pcl::PointCloud<pcl::PointXYZ>::Ptr &protopoints);
        float DisPointLine(const Eigen::Vector2f &line, const pcl::PointXYZ &point);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}

#endif // RR_PERCEPTION_RR_OBJECT_DETECTION_OBSTACLE_FILTER_H_
