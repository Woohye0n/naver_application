#include "rr_object_tracker/object_tracker.h"
#include "rr_object_tracker/sort_tracker.h"

namespace rr_perception
{

    struct ObjectTracker::Impl
    {
        SORTTracker tracker;
        ros::Publisher pub_trk;
        ros::Subscriber sub_det;
        ros::Time prev_stamp;
    };

    ObjectTracker::ObjectTracker() : impl_(new Impl) {}
    ObjectTracker::~ObjectTracker() {}
    void ObjectTracker::Init(ros::NodeHandle &nh)
    {
        // parameter variables
        int max_age;
        int min_hits;
        float iou_threshold;

        // get tracker parameters
        ROS_ASSERT(nh.getParam("max_age", max_age));
        ROS_ASSERT(nh.getParam("min_hits", min_hits));
        ROS_ASSERT(nh.getParam("iou_threshold", iou_threshold));

        // init tracker
        impl_->tracker.Init(max_age, min_hits, iou_threshold);

        // publisher
        impl_->pub_trk = nh.advertise<rr_common::PerceptionObstacleArray>("tracking", 3);
        impl_->sub_det = nh.subscribe("detection", 3, &ObjectTracker::Callback, this);
    }

    void ObjectTracker::Callback(const rr_common::PerceptionObstacleArray::Ptr &msg)
    {

        std::vector<OBB> detections;
        detections.reserve(msg->obstacles.size());
        std_msgs::Header header;
        header.stamp = ros::Time(0);
        float dt;
        for (auto &obs : msg->obstacles)
        {
            OBB bb;
            // find center position
            Eigen::Vector2f c;
            c << obs.pose.position.x, obs.pose.position.y;

            // find theta
            const float &s_th_2 = obs.pose.orientation.z;
            const float &c_th_2 = obs.pose.orientation.w;
            const float &th = atan2(s_th_2, c_th_2) * 2.0;

            // make rotation matrix
            const float &c_th = cos(th);
            const float &s_th = sin(th);
            Eigen::Matrix2f rot;
            rot << c_th, -s_th, s_th, c_th;

            // find pre rotation points
            const float &w = obs.shape.dimensions[0];
            const float &w_2 = w / 2.0;
            const float &h = obs.shape.dimensions[1];
            const float &h_2 = h / 2.0;
            Eigen::MatrixXf ps; // << pivot, p0, p1
            ps.resize(2, 3);
            ps << w_2, -w_2, w_2,
                h_2, h_2, -h_2;

            // rotate points
            Eigen::MatrixXf new_mat = rot * ps;

            // translate points
            new_mat.colwise() += c;

            bb.p_pivot = new_mat.block(0, 0, 2, 1);
            bb.p0 = new_mat.block(0, 1, 2, 1);
            bb.p1 = new_mat.block(0, 2, 2, 1);
            detections.push_back(bb);
            if (header.stamp == ros::Time(0))
            {
                header = obs.header;
                dt = (obs.header.stamp - impl_->prev_stamp).toSec();
                impl_->prev_stamp = obs.header.stamp;
            }
        }

        // update sort tracker
        impl_->tracker.SetDetections(detections);
        impl_->tracker.Update();

        // make a new message
        rr_common::PerceptionObstacleArray new_msg;

        auto &result = impl_->tracker.GetResult();
        for (auto &trk : result)
        {
            // temporal variables
            OBB bb;
            rr_common::PerceptionObstacle ob;

            ob.id = trk->GetId(); // set id
            ob.header = header;   // set header

            // make center point
            ConvertXToBox(trk->GetState(), bb);
            const auto &c = GetCenter(bb);
            ob.pose.position.x = c(0);
            ob.pose.position.y = c(1);
            ob.pose.position.z = 0.0;

            // make orientation
            ob.pose.orientation.x = 0.;
            ob.pose.orientation.y = 0.;
            ob.pose.orientation.z = sin(trk->GetState().x(2) / 2.0);
            ob.pose.orientation.w = cos(trk->GetState().x(2) / 2.0);

            // make shape
            const float &s = trk->GetState().x(3);
            const float &r = trk->GetState().x(4);
            if (s <= 0 || r <= 0)
            {
                continue;
            }
            const float &w = sqrt(s * r);
            const float &h = s / w;
            ob.shape.type = ob.shape.BOX;
            ob.shape.dimensions.push_back(w);
            ob.shape.dimensions.push_back(h);
            ob.shape.dimensions.push_back(1);

            // make velocity
            ob.twist.linear.x = trk->GetState().x(5) / dt;
            ob.twist.linear.y = trk->GetState().x(6) / dt;
            ob.twist.linear.z = trk->GetState().x(7) / dt;
            ob.twist.angular.x = 0.0;
            ob.twist.angular.y = 0.0;
            ob.twist.angular.z = trk->GetState().x(7) / dt;

            ROS_DEBUG_STREAM("[ pos ] [" << ob.id << "] : " << ob.pose.position.x << ", " << ob.pose.position.y << ", " << ob.pose.position.z);
            ROS_DEBUG_STREAM("[ vel ] [" << ob.id << "] : " << ob.twist.linear.x << ", " << ob.twist.linear.y << ", " << ob.twist.linear.z);
            ROS_DEBUG_STREAM("[w x h] [" << ob.id << "] : " << w << ", " << h);
            new_msg.obstacles.push_back(ob);
        }
        // publish

        impl_->pub_trk.publish(new_msg);
    }

}