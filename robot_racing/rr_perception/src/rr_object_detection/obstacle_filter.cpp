#include "rr_object_detection/obstacle_filter.h"

#include <pcl/segmentation/extract_clusters.h>

#include <ros/ros.h>

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

    struct ObstacleFilter::Impl
    {
        float lidar_height;
        float seg_deg;
        int seg_num;
        int bin_num;
        float bin_range;
        float max_dis;

        float T_m;
        float T_b;
        float T_th;
        float T_d;
    };
    ObstacleFilter::ObstacleFilter() : impl_(new Impl)
    {
    }
    ObstacleFilter::~ObstacleFilter()
    {
    }
    void ObstacleFilter::Init(float lidar_height,
                              float seg_deg,
                              int bin_num,
                              float max_dis,
                              float T_m,
                              float T_b,
                              float T_th,
                              float T_d)
    {
        impl_->lidar_height = lidar_height;
        impl_->seg_deg = seg_deg;
        impl_->seg_num = 360 / seg_deg;
        impl_->bin_num = bin_num;
        impl_->max_dis = max_dis;
        impl_->bin_range = max_dis / bin_num;
        impl_->T_m = T_m;
        impl_->T_b = T_b;
        impl_->T_th = T_th;
        impl_->T_d = T_d;
    }

    float ObstacleFilter::DisPointLine(const Eigen::Vector2f &line, const pcl::PointXYZ &point)
    {
        const float &p0 = sqrt(pow(point.x, 2) + pow(point.y, 2));
        const float &p1 = point.z;
        const float &dis = (line[0] * p0 - p1 + line[1]) / sqrt(pow(line[0], 2) + 1);
        return dis;
    }

    void ObstacleFilter::Filter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr pc_prj,
                                pcl::PointCloud<pcl::PointXYZL>::Ptr pc_pos_out)
    {
        // initialize output variables
        pc_pos_out->clear();
        pc_prj->clear();
        pc_pos_out->header = pc_in->header;
        pc_prj->header = pc_in->header;

        // allocate pcl vector for segmentation
        std::vector<pcl::PointIndices> *cluster_indices(new std::vector<pcl::PointIndices>(impl_->seg_num));

        // make segment by dividing azimuth angle
        pcl::PointCloud<pcl::PointXYZ>::iterator iter = pc_in->begin();
        for (int i = 0; iter != pc_in->end(); i++, iter++)
        {
            // set maximum value
            if (pow(iter->x, 2) + pow(iter->y, 2) > pow(impl_->max_dis, 2))
            {
                continue;
            }
            // calculate azimuth and push to segment
            float theta = atan2(iter->x, -iter->y) * 180 / M_PI - 90;
            theta = (theta < 0) ? theta + 360 : theta;

            // add points to their segmentation index
            int seg_idx = theta / impl_->seg_deg;
            seg_idx = (seg_idx == impl_->seg_num) ? seg_idx - 1 : seg_idx;
            cluster_indices->at(seg_idx).indices.push_back(i);
        }

        // make bin with range 0.1m and extract proto points for one segment
        for (auto &seg : *cluster_indices)
        {
            // proto points to extract line from one segmentation
            pcl::PointIndices::Ptr protopoints(new pcl::PointIndices());
            std::vector<pcl::PointIndices> *bins = new std::vector<pcl::PointIndices>(impl_->bin_num);

            // add base point of lidar to proto points
            pcl::PointXYZ base_point(0., 0., -impl_->lidar_height);
            pc_in->push_back(base_point);
            protopoints->indices.push_back(pc_in->size() - 1);

            // make bin vector for one segment
            for (auto &point : seg.indices)
            {
                // mapping points to each bin by calculating distance
                // 2D mapped point [sqrt(x^2 + y^2), 0, z]
                float dis = sqrt(pow(pc_in->at(point).x, 2) +
                                 pow(pc_in->at(point).y, 2) +
                                 pow(pc_in->at(point).z, 2));
                int bin_idx = dis / impl_->bin_range;
                bin_idx = (bin_idx >= impl_->bin_num) ? impl_->bin_num - 1 : bin_idx;

                // push to bin vector
                bins->at(bin_idx).indices.push_back(point);
            }

            // extract protopoints from bin
            auto comp = ZLessIndexComparator<pcl::PointXYZ>(pc_in);
            for (auto &bin : *bins)
            {
                if (bin.indices.size() != 0)
                {
                    // add lowest z-value point to proto points
                    std::sort(bin.indices.begin(), bin.indices.end(), comp);
                    protopoints->indices.push_back(*(bin.indices.begin()));
                }
            }

            // line (m, b) : slope and y-intercept
            Eigen::Vector2f line(0, -impl_->lidar_height);

            // set for line extraction
            pcl::PointIndices::Ptr line_points(new pcl::PointIndices());
            bool init = true;

            // extract line for one segment
            for (int i = 0; i < protopoints->indices.size(); i++)
            {
                if (line_points->indices.size() >= 2)
                {
                    const int &size = line_points->indices.size();
                    Eigen::MatrixXf A(size, 2);
                    Eigen::MatrixXf B(size, 1);
                    Eigen::Vector2f X;

                    for (int i = 0; i < size; i++)
                    {
                        A(i, 0) = sqrt(pow((pc_in->at(line_points->indices.at(i))).x, 2) +
                                       pow((pc_in->at(line_points->indices.at(i))).y, 2));
                        B(i, 0) = (pc_in->at(line_points->indices.at(i))).z;
                        A(i, 1) = 1;
                    }

                    // find slope and y-intercept with Least Square Meathod
                    X = ((A.transpose() * A).inverse()) * A.transpose() * B;

                    // calculate degree between test line and reference line
                    Eigen::Vector2f test_line(X[0], 1);
                    Eigen::Vector2f ref_line(line[0], 1);
                    test_line.normalize();
                    ref_line.normalize();
                    float deg = acos(test_line.dot(ref_line)) * 180 / M_PI;

                    // check if fitted line is grond
                    if (deg < impl_->T_m && fabs(line[1] - X[1]) < impl_->T_b)
                    {
                        line_points->indices.push_back(protopoints->indices[i]);
                        line = X;
                    }
                }
                else
                {
                    if (fabs(DisPointLine(line, pc_in->at(protopoints->indices[i]))) < impl_->T_d &&
                        pc_in->at(protopoints->indices[i]).z < -0.5)
                    {
                        line_points->indices.push_back(protopoints->indices[i]);
                    }
                }
            }
            // set non-ground points which is not on ground line
            for (auto &point : seg.indices)
            {
                if (DisPointLine(line, pc_in->at(point)) < -impl_->T_th)
                {
                    pcl::PointXYZL labeled_point;
                    labeled_point.x = pc_in->at(point).x;
                    labeled_point.y = pc_in->at(point).y;
                    labeled_point.z = pc_in->at(point).z;
                    labeled_point.label = 0;

                    pc_prj->push_back(pc_in->at(point));
                    pc_pos_out->push_back(labeled_point);
                }
            }
        }
    }

}