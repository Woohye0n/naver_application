#include "rr_strategy/obstacle_checker.h"
#include "rr_strategy/current_course_checker.h"
#include "rr_strategy/rr_strategy.h"

namespace rr_strategy
{
    struct Strategy::Impl
    {
        rr_common::StrategyMode mode;
        rr_common::SpeedControlTarget target_speed;

        ros::Subscriber sub_state;
        ros::Subscriber sub_collision;
        ros::Subscriber sub_obstacle;

        ros::Publisher pub_strategy;
        ros::Publisher pub_target_speed;

        // racing parameters
        std::vector<rr_common::PointXY> racing;
        std::vector<rr_common::PointXY> highway_in;
        std::vector<rr_common::PointXY> highway_out;

        // current state
        rr_common::PointXY cur_xy;
        float heading;
        float speed;

        bool is_collide;
        float collision_time;

        int cur_course;
        int strategy_course;

        ObstacleChecker obstacle_checker;
        CurrentCourseChecker current_course_checker;

        float ttc = 0;
    };

    Strategy::Strategy() : impl_(new Impl) {}
    Strategy::~Strategy() {}

    void Strategy::Init(ros::NodeHandle &nh)
    {
        impl_->obstacle_checker.Init(nh);
        impl_->current_course_checker.Init(nh);

        ///
        //!@brief activate service, subscribe, publish
        ///
        ros::ServiceClient client = nh.serviceClient<rr_common::MapWayPointXY>("/map_xy_points");
        rr_common::MapWayPointXY srv;
        srv.request.header.stamp = ros::Time::now();

        bool init = true;

        while (init)
        {
            if (client.call(srv))
            {
                impl_->racing = srv.response.racing;
                impl_->highway_in = srv.response.highway_in;
                impl_->highway_out = srv.response.highway_out;
                init = false;
            }
        }

        // set map
        impl_->obstacle_checker.SetMap(impl_->highway_in,
                                       impl_->highway_out);
        impl_->current_course_checker.SetMap(impl_->highway_in,
                                             impl_->highway_out);

        // publisher
        impl_->pub_strategy = nh.advertise<rr_common::StrategyMode>("strategy", 1);
        impl_->pub_target_speed = nh.advertise<rr_common::SpeedControlTarget>("target_speed", 3);

        impl_->target_speed.header.frame_id = "rr_strategy";

        // subscriber
        impl_->sub_state = nh.subscribe("state", 3, &Strategy::StateCb, this);
        impl_->sub_collision = nh.subscribe("/rr_object_detection/time_to_collision", 3, &Strategy::CollisionCb, this);
        impl_->sub_obstacle = nh.subscribe("obstacle", 3, &Strategy::ObstacleCb, this);
    }

    void Strategy::StateCb(const rr_common::StateEstimated::Ptr &state)
    {
        impl_->obstacle_checker.SetState(state->current_xy);
        impl_->obstacle_checker.SetHeading(state->heading);
        impl_->current_course_checker.SetState(state->current_xy);
        ROS_DEBUG("[STRATEGY] : state callback ");
    }

    void Strategy::CollisionCb(const rr_common::TimeToCollision::Ptr &collision)
    {
        impl_->ttc = collision->time_to_collision;
        ROS_DEBUG("[STRATEGY] :collision callback ");
    }

    void Strategy::ObstacleCb(const rr_common::PerceptionObstacleArray::Ptr &obstacle)
    {
        impl_->obstacle_checker.SetObstacle(obstacle);
        ROS_DEBUG("[STRATEGY] : obstacle callback");
    }

    void Strategy::SpinOnce()
    {
        rr_common::StrategyMode mode;

        if (impl_->current_course_checker.IsOk() && impl_->obstacle_checker.IsOk())
        {
            impl_->current_course_checker.FindCurrentCourse();
            impl_->cur_course = impl_->current_course_checker.GetCurrentCourse();

            impl_->obstacle_checker.SetCurrentCourse(impl_->cur_course);

            impl_->obstacle_checker.MakeStrategy();

            impl_->strategy_course = impl_->obstacle_checker.GetCourse();

            impl_->mode.course = impl_->strategy_course;

            if (impl_->strategy_course == 0)
            {
                impl_->target_speed.speed = 0;
            }

            impl_->target_speed.header.stamp = ros::Time::now();
            impl_->pub_target_speed.publish(impl_->target_speed);
            impl_->pub_strategy.publish(impl_->mode);
        }
    }
}
