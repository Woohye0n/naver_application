#include "rr_strategy/obstacle_checker.h"

namespace rr_strategy
{
    struct ObstacleChecker::Impl
    {
        // obstacle vector in map coordiate
        std::vector<Obstacle> obstacles;
        std::vector<Obstacle> course1_obstacles;
        std::vector<Obstacle> course2_obstacles;
        bool position_set = false;

        // obstacle vector in robot coordinate
        std::vector<Obstacle> robot_obstacles;

        float stop_dis;
        bool stop = false;

        int current_course;

        int strategy_course;
        int before_zero;
        int prev_strategy_course;

        std::vector<rr_common::PointXY> map_in;
        std::vector<rr_common::PointXY> map_out;

        rr_common::PointXY position;
        float theta; // rad
    };

    ObstacleChecker::ObstacleChecker() : impl_(new Impl) {}
    ObstacleChecker::~ObstacleChecker() {}

    void ObstacleChecker::Init(ros::NodeHandle &nh)
    {
        ROS_ASSERT(nh.getParam("stop_distance", impl_->stop_dis));
    }

    bool ObstacleChecker::IsOk()
    {
        // OK if position is set
        if (impl_->position_set)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void ObstacleChecker::SetState(const rr_common::PointXY &state)
    {
        impl_->position = state;
        impl_->position_set = true;
    }

    void ObstacleChecker::SetHeading(const float &heading)
    {
        impl_->theta = (90 - heading) * M_PI / 180;
    }

    void ObstacleChecker::SetObstacle(const rr_common::PerceptionObstacleArray::Ptr &obstacle_array)
    {

        impl_->obstacles.clear();
        impl_->robot_obstacles.clear();
        float r_x = 0;
        float r_y = 0;
        int i = 0;

        // transform to map coordinate
        for (auto &ob : obstacle_array->obstacles)
        {
            // push back if obstacle is in front
            if (ob.pose.position.x > 0)
            {
                Obstacle obstacle;
                // push back to robot coordinate obstacle
                obstacle.x = ob.pose.position.x;
                obstacle.y = ob.pose.position.y;
                obstacle.idx = i;
                impl_->robot_obstacles.push_back(obstacle);

                // push back to map coordinte obstacle
                obstacle.x = impl_->position.x +
                             cos(impl_->theta) * ob.pose.position.x -
                             sin(impl_->theta) * ob.pose.position.y;
                obstacle.y = impl_->position.y +
                             sin(impl_->theta) * ob.pose.position.x +
                             cos(impl_->theta) * ob.pose.position.y;
                impl_->obstacles.push_back(obstacle);
                i++;
            }
        }
    }

    void ObstacleChecker::SetCurrentCourse(int current_course)
    {
        impl_->current_course = current_course;
    }

    void ObstacleChecker::SetMap(const std::vector<rr_common::PointXY> &map_in, const std::vector<rr_common::PointXY> &map_out)
    {
        impl_->map_in = map_in;
        impl_->map_out = map_out;
    }

    int ObstacleChecker::GetCourse()
    {
        return impl_->strategy_course;
    }

    void ObstacleChecker::SelectObstacle()
    {
        impl_->stop = false;
        impl_->course1_obstacles.clear();
        impl_->course2_obstacles.clear();

        // if obstacle is in 3m in front of robot, stop
        for (auto &obstacle : impl_->obstacles)
        {
            if (impl_->robot_obstacles[obstacle.idx].x < impl_->stop_dis &&
                fabs(impl_->robot_obstacles[obstacle.idx].y) < 1)
            {
                impl_->stop = true;
                ROS_DEBUG("distance %f %f", impl_->robot_obstacles[obstacle.idx].x, impl_->robot_obstacles[obstacle.idx].y);
                return;
            }
        }

        // pruning obstacles near each course and push back to each vector
        for (auto &obstacle : impl_->obstacles)
        {
            // pruning if obstacle is in 10m
            if (sqrt(pow((obstacle.x - impl_->position.x), 2) + pow((obstacle.y - impl_->position.y), 2)) < 10)
            {
                for (auto &map : impl_->map_in)
                {
                    // if obstacle is near course 1
                    if (sqrt(pow((obstacle.x - map.x), 2) + pow((obstacle.y - map.y), 2)) < 1)
                    {
                        impl_->course1_obstacles.push_back(obstacle);
                        ROS_DEBUG("course 1 obs %f, %f : ", obstacle.x, obstacle.y);
                    }
                }

                for (auto &map : impl_->map_out)
                {
                    // if obstacle is near course 2
                    if (sqrt(pow((obstacle.x - map.x), 2) + pow((obstacle.y - map.y), 2)) < 1)
                    {
                        impl_->course2_obstacles.push_back(obstacle);
                        ROS_DEBUG("course 2 obs %f, %f : ", obstacle.x, obstacle.y);
                    }
                }
            }
        }
        ROS_DEBUG("course 1 obstacle num : %d", impl_->course1_obstacles.size());
        ROS_DEBUG("course 2 obstacle num : %d", impl_->course2_obstacles.size());
    }

    void ObstacleChecker::MakeStrategy()
    {
        SelectObstacle();

        // if obstacle is too close, stop
        if (impl_->stop)
        {
            impl_->strategy_course = 0;
            return;
        }

        // if robot is on course 1
        if (impl_->current_course == 1)
        {
            // if obstacle is on couse 1 and course 2
            if (impl_->course1_obstacles.size() != 0 && impl_->course2_obstacles.size() != 0)
            {
                impl_->strategy_course = 0;
            }
            // if obstacle is on couse 1 and not on course 2
            else if (impl_->course1_obstacles.size() != 0 && impl_->course2_obstacles.size() == 0)
            {
                impl_->strategy_course = 12;
            }
            // if obstacle is not on couse 1 and on course 2
            else if (impl_->course1_obstacles.size() == 0 && impl_->course2_obstacles.size() != 0)
            {
                impl_->strategy_course = 1;
            }
            // if obstacle is in not on couse 1 and course 2
            else if (impl_->course1_obstacles.size() == 0 && impl_->course2_obstacles.size() == 0)
            {
                impl_->strategy_course = 1;
            }
        }
        // if robot is on course 2
        else
        {
            // if obstacle is on couse 1 and course 2
            if (impl_->course1_obstacles.size() != 0 && impl_->course2_obstacles.size() != 0)
            {
                impl_->strategy_course = 0;
            }
            // if obstacle is on couse 1 and not on course 2
            else if (impl_->course1_obstacles.size() != 0 && impl_->course2_obstacles.size() == 0)
            {
                impl_->strategy_course = 2;
            }
            // if obstacle is not on couse 1 and on course 2
            else if (impl_->course1_obstacles.size() == 0 && impl_->course2_obstacles.size() != 0)
            {
                impl_->strategy_course = 21;
            }
            // if obstacle is in not on couse 1 and course 2
            else if (impl_->course1_obstacles.size() == 0 && impl_->course2_obstacles.size() == 0)
            {
                impl_->strategy_course = 21;
            }
        }
        IsChanged();
    }

    void ObstacleChecker::IsChanged()
    {
        if (impl_->strategy_course == 12)
        {
            for (auto &map : impl_->map_out)
            {
                // if current position is near course 2
                if (sqrt(pow((impl_->position.x - map.x), 2) + pow((impl_->position.y - map.y), 2)) <= 1)
                {
                    impl_->strategy_course = 2;
                    return;
                }
            }
        }

        else if (impl_->strategy_course == 21)
        {
            for (auto &map : impl_->map_in)
            {
                // if current position is near course 1
                if (sqrt(pow((impl_->position.x - map.x), 2) + pow((impl_->position.y - map.y), 2)) <= 1)
                {
                    impl_->strategy_course = 1;
                    return;
                }
            }
        }
    }

} // namespace rr_strategy
