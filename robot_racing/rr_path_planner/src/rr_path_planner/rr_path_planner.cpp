#include "rr_path_planner/path_generator.h"
#include "rr_path_planner/heading_checker.h"
#include "rr_path_planner/on_course_checker.h"
#include "rr_path_planner/speed_generator.h"
#include "rr_path_planner/rr_path_planner.h"
#include "rr_common/MapWayPointXY.h"

namespace rr_path_planner
{

    struct LocalPathPlanner::Impl
    {
        rr_common::LocalPathPoints local_path_points;
        rr_common::SpeedControlTarget target_speed;

        ros::Publisher pub_local_path;
        ros::Publisher pub_target_speed;

        ros::Subscriber sub_state;
        ros::Subscriber sub_course;

        std::vector<rr_common::PointXY> map_in;
        std::vector<rr_common::PointXY> map_out;

        std::vector<rr_common::PointXY> local_path;

        HeadingChecker heading_check;
        OnCourseChecker on_course_check;
        PathGenerator path_generate;
        SpeedGenerator speed_generate;

        bool init = false;
    };

    LocalPathPlanner::LocalPathPlanner() : impl_(new Impl) {}
    LocalPathPlanner::~LocalPathPlanner() {}

    void LocalPathPlanner::Init(ros::NodeHandle &nh)
    {
        // get map xy points service
        ros::ServiceClient client =
            nh.serviceClient<rr_common::MapWayPointXY>("/map_xy_points");
        rr_common::MapWayPointXY srv;
        srv.request.header.stamp = ros::Time::now();

        // wait until response arrive
        bool init = true;

        while (init)
        {
            if (client.call(srv))
            {
                impl_->map_in = srv.response.highway_in;
                impl_->map_out = srv.response.highway_out;
                init = false;
            }
        }

        impl_->pub_local_path = nh.advertise<rr_common::LocalPathPoints>("local_path", 3);
        impl_->pub_target_speed = nh.advertise<rr_common::SpeedControlTarget>("target_speed", 3);

        // Init
        impl_->heading_check.Init(nh, "deg_diff_allowed");
        impl_->on_course_check.Init(nh, "path_diff_allowed");
        impl_->path_generate.Init(nh, "buffer_size");
        impl_->speed_generate.Init(nh);

        // Set Map
        impl_->path_generate.SetMap(impl_->map_in, impl_->map_out);

        impl_->sub_state = nh.subscribe("state", 3, &LocalPathPlanner::StateCallback, this);
        impl_->sub_course = nh.subscribe("course", 3, &LocalPathPlanner::StrategyCallback, this);
    }

    void LocalPathPlanner::StateCallback(const rr_common::StateEstimated::Ptr &state)
    {

        impl_->heading_check.SetHeading(state->heading);
        impl_->heading_check.SetState(state->current_xy);
        impl_->path_generate.SetState(state->current_xy);
        impl_->on_course_check.SetState(state->current_xy);
        impl_->speed_generate.SetState(state->current_xy);
    }

    void LocalPathPlanner::StrategyCallback(const rr_common::StrategyMode::Ptr &course)
    {
        impl_->path_generate.SetCourse(course->course);
    }

    void LocalPathPlanner::SpinOnce()
    {
        impl_->local_path.clear();

        if (!impl_->init)
        {
            impl_->path_generate.ClearPath();
            impl_->path_generate.CreateLocalPath();
            impl_->local_path = impl_->path_generate.GetPath();
            impl_->init = true;
            if (impl_->local_path.size() < 30)
            {
                impl_->init = false;
            }
        }

        if (impl_->init)
        {
            impl_->on_course_check.SetPath(impl_->local_path);
            impl_->heading_check.SetPath(impl_->local_path);

            if (impl_->heading_check.IsOk() && impl_->on_course_check.IsOk())
            {
                impl_->path_generate.CreateLocalPath();
                impl_->local_path = impl_->path_generate.GetPath();
            }
            else
            {
                // clear
                impl_->path_generate.ClearPath();
                impl_->path_generate.CreateLocalPath();
                impl_->local_path = impl_->path_generate.GetPath();
            }

            impl_->on_course_check.SetPath(impl_->local_path);
            impl_->heading_check.SetPath(impl_->local_path);

            // calculate curvature of path and get speed
            impl_->speed_generate.SetPath(impl_->local_path);
            impl_->speed_generate.MakeVector();

            // pub target speed
            impl_->target_speed.speed = impl_->speed_generate.GetSpeed();
            impl_->target_speed.header.frame_id = "rr_path_planner";
            impl_->target_speed.header.stamp = ros::Time::now();
            impl_->pub_target_speed.publish(impl_->target_speed);

            // pub local path
            impl_->local_path_points.local_path_points = impl_->local_path;
            impl_->pub_local_path.publish(impl_->local_path_points);
        }
    }
}