#include "rr_object_detection/bottom_prune.h"

namespace rr_perception
{
    template <typename PointT>
    class ZLessIndexComparator
    {
    public:
        ZLessIndexComparator(const typename pcl::PointCloud<PointT>::Ptr &ptr) : pc_(ptr){};
        bool operator()(const int &left, const int &right)
        {
            return pc_->at(left).z < pc_->at(right).z;
        }

    private:
        typename pcl::PointCloud<PointT>::Ptr pc_;
    };

    struct BottomPrune::Impl
    {
        float pruning_height;
    };
    BottomPrune::BottomPrune() : impl_(new Impl) {}
    BottomPrune::~BottomPrune() {}
    void BottomPrune::Init(float pruning_height)
    {
        impl_->pruning_height = pruning_height;
    }
    void BottomPrune::Filter(pcl::PointCloud<pcl::PointXYZL>::Ptr pc_in,
                             std::vector<pcl::PointIndices> &indicies_in,
                             std::vector<pcl::PointIndices::Ptr> &indicies_out)
    {
        if (pc_in->size() == 0)
        {
            return;
        }
        indicies_out.clear();
        auto comp = ZLessIndexComparator<pcl::PointXYZL>(pc_in);

        for (auto &c_idcs : indicies_in)
        {
            pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
            std::sort(c_idcs.indices.begin(), c_idcs.indices.end(), comp);
            float prune_threshold = pc_in->at(c_idcs.indices[0]).z + impl_->pruning_height;
            for (auto &idx : c_idcs.indices)
            {
                if (pc_in->at(idx).z < prune_threshold)
                {
                    continue;
                }
                cluster_indices->indices.push_back(idx);
            }
            indicies_out.push_back(cluster_indices);
        }
    }

} // namespace rr_perception
