#ifndef RR_PERCEPTION_RR_OBJECT_DETECTION_BOTTOM_PRUNE_H_
#define RR_PERCEPTION_RR_OBJECT_DETECTION_BOTTOM_PRUNE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include <memory>

namespace rr_perception
{
    class BottomPrune
    {
    public:
        BottomPrune();
        ~BottomPrune();
        void Init(float pruning_height);
        void Filter(pcl::PointCloud<pcl::PointXYZL>::Ptr pc_in,
                    std::vector<pcl::PointIndices> &indicies_in,
                    std::vector<pcl::PointIndices::Ptr> &indicies_out);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

};

#endif //RR_PERCEPTION_RR_OBJECT_DETECTION_BOTTOM_PRUNE_H_