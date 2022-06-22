#include "rr_safety_manager/rr_course_out_detector.h"

namespace rr_safety_manager
{

    struct CourseOutDetector::Impl
    {
        ros::ServiceClient map_client;
        ros::Subscriber sub_state;

        // map data
        std::vector<rr_common::PointXY> highway_in_points;
        std::vector<rr_common::PointXY> highway_out_points;
        int in_size;
        int out_size;

        // current state
        rr_common::PointXY cur_xy;

        // param
        float out_check_comp;
    };
    CourseOutDetector::CourseOutDetector() : impl_(new Impl) {}
    CourseOutDetector::~CourseOutDetector() {}

    bool CourseOutDetector::IsOut()
    {
        rr_common::PointXY temp_in = FindClosestPoint(impl_->highway_in_points, impl_->cur_xy);
        rr_common::PointXY temp_out = FindClosestPoint(impl_->highway_out_points, impl_->cur_xy);

        return (CalDistance(impl_->cur_xy, temp_in) > impl_->out_check_comp && CalDistance(impl_->cur_xy, temp_out) > impl_->out_check_comp);
    }

    void CourseOutDetector::Init(ros::NodeHandle &nh)
    {
        impl_->highway_in_points.clear();
        impl_->highway_out_points.clear();
        GetMapXY(nh);
        impl_->sub_state = nh.subscribe("state", 1, &CourseOutDetector::StateCb, this);
        ROS_ASSERT(nh.getParam("out_check_comp", impl_->out_check_comp));
    }

    void CourseOutDetector::GetMapXY(ros::NodeHandle &nh)
    {
        // request service
        impl_->map_client = nh.serviceClient<rr_common::MapWayPointXY>("/map_xy_points");
        rr_common::MapWayPointXY srv;
        srv.request.header.stamp = ros::Time::now();

        // wait until response arrive
        bool init = true;
        ROS_INFO("Waiting for service response");

        while (init)
        {
            if (impl_->map_client.call(srv))
            {
                impl_->highway_in_points = srv.response.highway_in;
                impl_->highway_out_points = srv.response.highway_out;
                init = false;
                ROS_INFO("Service Recieved");
            }
        }

        impl_->in_size = impl_->highway_in_points.size();
        impl_->out_size = impl_->highway_out_points.size();
    }

    void CourseOutDetector::StateCb(const rr_common::StateEstimated::Ptr &state_sub)
    {
        impl_->cur_xy.x = state_sub->current_xy.x;
        impl_->cur_xy.y = state_sub->current_xy.y;
    }

    rr_common::PointXY CourseOutDetector::FindClosestPoint(const std::vector<rr_common::PointXY> &way_points, rr_common::PointXY p)
    {
        rr_common::PointXY closest;
        float distance;
        float last_distance;
        float min = CalDistance(way_points[0], p);

        for (auto &temp : way_points)
        {
            distance = CalDistance(temp, p);
            if (distance <= min)
            {
                min = distance;
                closest.x = temp.x;
                closest.y = temp.y;
                closest.idx = temp.idx % way_points.size();
            }
        }

        return closest;
    }

    float CalDistance(rr_common::PointXY point1, rr_common::PointXY point2)
    {
        return sqrt(pow((point1.x - point2.x), 2) + pow((point1.y - point2.y), 2));
    }
} // namespace rr_safety_manager
