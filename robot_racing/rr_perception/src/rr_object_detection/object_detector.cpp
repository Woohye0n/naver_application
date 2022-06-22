
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include "rr_object_detection/object_detector.h"
#include "rr_object_detection/obstacle_filter.h"
#include "rr_object_detection/obstacle_cluster_extractor.h"
#include "rr_object_detection/bottom_prune.h"
#include "rr_common/PerceptionObstacleArray.h"
#include "rr_common/TimeToCollision.h"
// debug
#include <pcl/features/moment_of_inertia_estimation.h>

#include <shape_msgs/Plane.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

namespace rr_perception
{
    struct ObjectDetector::Impl
    {
        ros::Subscriber sub;
        ros::Subscriber sub_feedback;
        ros::Publisher pub_ob;
        ros::Publisher pub_detection;
        ros::Publisher pub_ttc;

        ObstacleFilter ob_filter;
        ObstacleClusterExtractor ob_cluster_extractor;
        rr_common::ERP42FeedbackExt feedback;
        BottomPrune bt_prune;
        float front_dist;
    };

    ObjectDetector::ObjectDetector(ros::NodeHandle &nh) : impl_(new Impl)
    {
        // initialize subscriber
        impl_->sub = nh.subscribe("input", 3, &ObjectDetector::Callback, this);
        impl_->sub_feedback = nh.subscribe("feedback", 3, &ObjectDetector::FeedbackCallback, this);
        // initialize publishers
        impl_->pub_ob = nh.advertise<sensor_msgs::PointCloud2>("non_ground_points", 3);
        impl_->pub_detection = nh.advertise<rr_common::PerceptionObstacleArray>("detection", 3);
        // debug
        // time to collision
        impl_->pub_ttc = nh.advertise<rr_common::TimeToCollision>("time_to_collision", 3);

        // load obstacle filter parameters
        float lidar_height;
        float seg_deg;
        int bin_num;
        float max_dis;
        float T_m;
        float T_b;
        float T_th;
        float T_d;

        ROS_ASSERT(nh.getParam("lidar_height", lidar_height));
        ROS_ASSERT(nh.getParam("seg_deg", seg_deg));
        ROS_ASSERT(nh.getParam("bin_num", bin_num));
        ROS_ASSERT(nh.getParam("max_dis", max_dis));
        ROS_ASSERT(nh.getParam("T_m", T_m));
        ROS_ASSERT(nh.getParam("T_b", T_b));
        ROS_ASSERT(nh.getParam("T_th", T_th));
        ROS_ASSERT(nh.getParam("T_d", T_d));
        impl_->ob_filter.Init(lidar_height,
                              seg_deg,
                              bin_num,
                              max_dis,
                              T_m,
                              T_b,
                              T_th,
                              T_d);

        // load obstacle cluster extractor parameters
        int min_cluster_size;
        int max_cluster_size;
        int max_neighbor_distance;
        ROS_ASSERT(nh.getParam("min_cluster_size", min_cluster_size));
        ROS_ASSERT(nh.getParam("max_cluster_size", max_cluster_size));
        ROS_ASSERT(nh.getParam("max_neighbor_distance", max_neighbor_distance));
        impl_->ob_cluster_extractor.Init(min_cluster_size,
                                         max_cluster_size,
                                         max_neighbor_distance);

        // bottom pruning for each cluster
        float pruning_height;
        ROS_ASSERT(nh.getParam("pruning_height", pruning_height));
        impl_->bt_prune.Init(pruning_height);
        impl_->front_dist = 0;
    }

    ObjectDetector::~ObjectDetector() {}

    void ObjectDetector::Callback(const sensor_msgs::PointCloud2ConstPtr &input)
    {
        // Read message
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_input_vd(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZL>::Ptr pc_ob(new pcl::PointCloud<pcl::PointXYZL>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ob_prj(new pcl::PointCloud<pcl::PointXYZ>());
        std::vector<pcl::PointIndices> cluster_indices;
        std::vector<pcl::PointIndices::Ptr> pruned_indices;

        pcl::fromROSMsg(*input, *pc_input_vd);

        // voxelize pointcloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

        float voxelsize = 0.1;

        voxel_filter.setInputCloud(pc_input_vd);
        voxel_filter.setLeafSize(voxelsize, voxelsize, voxelsize);
        voxel_filter.filter(*pc_filtered);

        // filter obstacle
        impl_->ob_filter.Filter(pc_filtered, pc_ob_prj, pc_ob);

        // clustering
        impl_->ob_cluster_extractor.Extract(pc_ob_prj, pc_ob, cluster_indices);

        // bottom pruning
        impl_->bt_prune.Filter(pc_ob, cluster_indices, pruned_indices);

        // float min_dist = 100; // roi distance limit is 20 m. 40 is enough
        // rr_common::TimeToCollision ttc_msg;
        // const float &steer = impl_->feedback.steer_rad;
        // for (auto &idcs : pruned_indices)
        // {
        //     for (auto &idx : idcs->indices)
        //     {
        //         const float &x = pc_ob->at(idx).x;
        //         const float &y = pc_ob->at(idx).y;
        //         const float &z = pc_ob->at(idx).z;
        //         // roi is front, in 20 m range
        //         if (x < 0 || x * x + y * y > 400)
        //         {
        //             continue;
        //         }

        //         // no steering,
        //         if (fabs(steer) < 0.01)
        //         {
        //             // out of roi. side limit
        //             if ((y > 0.8) || (y < -0.8))
        //             {
        //                 continue;
        //             }
        //             min_dist = min_dist < x ? min_dist : x;
        //         }
        //         // steering
        //         else
        //         {
        //             const float R = 1.212 / tan(steer);
        //             const float &rc_sq = (x - R) * (x - R) + y * y;
        //             const float &R_r = R - 0.8;
        //             const float &R_l = R + 0.8;
        //             // out of roi. side limit
        //             if (rc_sq < R_r * R_r && rc_sq > R_l * R_l)
        //             {
        //                 continue;
        //             }
        //             // in roi
        //             const float &dist = R * steer;
        //             min_dist = min_dist < dist ? min_dist : dist;
        //         }
        //     }
        // }
        // const float &a = 0.7;
        // impl_->front_dist = a * impl_->front_dist + (1.0 - a) * min_dist;

        // if (impl_->front_dist < 20 && impl_->feedback.speed_mps > 0)
        // {

        //     ttc_msg.time_to_collision = impl_->front_dist / impl_->feedback.speed_mps;
        // }
        // else
        // {
        //     ttc_msg.time_to_collision = -1;
        // }

        // ROS_INFO_STREAM("TTC: " << ttc_msg.time_to_collision << ", min_dist: " << min_dist << ", speed: " << impl_->feedback.speed_mps);
        // impl_->pub_ttc.publish(ttc_msg);

        // distance 100
        rr_common::PerceptionObstacleArray msg_det;

        ROS_DEBUG("[ %d ] clusters found ", (int)cluster_indices.size());
        for (int i = 0; i < pruned_indices.size(); i++)
        {
            if (pruned_indices[i]->indices.size() == 0)
            {
                continue;
            }

            // variables for OBB
            pcl::PointXYZ min_point_OBB;
            pcl::PointXYZ max_point_OBB;
            pcl::PointXYZ position_OBB;
            Eigen::Matrix3f rotational_matrix_OBB;

            // get obb from projected points
            pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
            feature_extractor.setInputCloud(pc_ob_prj);
            feature_extractor.setIndices(pruned_indices[i]);
            feature_extractor.compute();
            feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

            Eigen::Vector3f scale = (max_point_OBB.getVector3fMap() - min_point_OBB.getVector3fMap());
            if (scale.norm() > 5)
            {
                continue;
            }
            // get max_z and min_z
            float max_z, min_z;
            max_z = min_z = pc_ob->at(cluster_indices[i].indices[0]).z;
            for (auto &idx : cluster_indices[i].indices)
            {
                const Eigen::Vector3f &vec = pc_ob->at(idx).getVector3fMap();
                max_z = vec(2) > max_z ? vec(2) : max_z;
                min_z = vec(2) < min_z ? vec(2) : min_z;
            }

            if (max_z - min_z < 0.1)
            {
                continue;
            }

            rr_common::PerceptionObstacle msg_det_one;
            msg_det_one.header.frame_id = input->header.frame_id;
            msg_det_one.header.stamp = input->header.stamp;
            msg_det_one.id = i;

            msg_det_one.pose.position.x = position_OBB.x;
            msg_det_one.pose.position.y = position_OBB.y;
            msg_det_one.pose.position.z = (max_z + min_z) / 2.0;

            Eigen::Quaternionf q(rotational_matrix_OBB);
            msg_det_one.pose.orientation.w = q.w();
            msg_det_one.pose.orientation.x = q.x();
            msg_det_one.pose.orientation.y = q.y();
            msg_det_one.pose.orientation.z = q.z();

            msg_det_one.shape.type = msg_det_one.shape.BOX;
            msg_det_one.shape.dimensions.push_back(max_point_OBB.x - min_point_OBB.x);
            msg_det_one.shape.dimensions.push_back(max_point_OBB.y - min_point_OBB.y);
            msg_det_one.shape.dimensions.push_back(max_z - min_z);
            msg_det.obstacles.push_back(msg_det_one);
        }
        impl_->pub_detection.publish(msg_det);

        // publish ground points.
        sensor_msgs::PointCloud2 msg_ob;
        pcl::toROSMsg(*pc_ob, msg_ob);
        impl_->pub_ob.publish(msg_ob);
        return;
    }
    void ObjectDetector::FeedbackCallback(const rr_common::ERP42FeedbackExt::Ptr &msg)
    {
        impl_->feedback = *msg;
    }

    void ObjectDetector::Run()
    {
        ros::spin();
    }
}
