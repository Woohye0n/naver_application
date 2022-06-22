
// Current Collision Safety

// course must in [1, 2]
//     1 : in_course
//     2 : out_course

// area must be in [1, 2, 3]
//     1 : Front
//     2 : Middle
//     3 : Back

// current xy : in_course and Middle (1, 2)

//         *** Collision Space ***
//  _____________________________________
// |                  |                  |
// |       1, 1       |       2, 1       |
// |__________________|__________________|
// |                  |                  |
// |      cur_xy_     |       2, 2       |
// |__________________|__________________|
// |                  |                  |
// |       1, 3       |       2, 3       |
// |__________________|__________________|

// 현재 차량을 주변으로 둘러싼 5개의 구역에 장애물이 있는지 없는지 확인 후
// rr_strategy로 collision space를 발행합니다.

#include "rr_collision_safety/rr_collision_safety.h"

namespace rr_collision_safety
{
    CollisionSafety::CollisionSafety()
    {
        GetMapXY();
        ROS_ASSERT(nh_.getParam("collision_comp", collision_comp_));
        pub_collision_safety_ = nh_.advertise<rr_common::CollisionSafety>("/rr_collision_safety/collision_safety", 1);

        Subscribe();
    }

    void CollisionSafety::Subscribe()
    {
        sub_state_ = nh_.subscribe("/rr_state_estimation/state", 1, &CollisionSafety::StateCb, this);
        sub_obs_ = nh_.subscribe("/rr_object_detection/detection", 1, &CollisionSafety::ObstacleCb, this);
    }

    void CollisionSafety::GetMapXY()
    {
        // request service
        map_client_ = nh_.serviceClient<rr_common::MapWayPointXY>("/map_xy_points");
        rr_common::MapWayPointXY srv;
        srv.request.header.stamp = ros::Time::now();

        // wait until response arrive
        bool init = true;
        ROS_INFO("Waiting for service response");

        while (init)
        {
            if (map_client_.call(srv))
            {
                highway_in_ = srv.response.highway_in;
                highway_out_ = srv.response.highway_out;
                init = false;
                ROS_INFO("Service Recieved");
            }
        }
        in_size_ = highway_in_.size();
        out_size_ = highway_out_.size();
    }

    void CollisionSafety::StateCb(const rr_common::StateEstimated::Ptr &state_sub)
    {
        cur_xy_.x = state_sub->current_xy.x;
        cur_xy_.y = state_sub->current_xy.y;
        heading_ = state_sub->heading;
        speed_ = state_sub->speed;
    }

    void CollisionSafety::ObstacleCb(const rr_common::PerceptionObstacleArray::Ptr &obstacles)
    {
        // Reset obstacles
        detected_obstacles_.clear();

        Obstacle temp;

        // Make a vector of detected obstacles
        for (auto &received_obs : obstacles->obstacles)
        {
            temp.id = received_obs.id;
            temp.pose = CoordConvert(received_obs.pose);
            temp.shape = received_obs.shape;
            temp.twist = received_obs.twist;

            detected_obstacles_.push_back(temp);
        }

        FigureValidObs();
        CollisionCheck();
    }

    void CollisionSafety::FigureValidObs()
    {
        detected_valid_.clear();

        // current의 가장 가까운 in, out 포인트
        auto nearest_cur_in = FindClosestPoint(highway_in_, cur_xy_);
        auto nearest_cur_out = FindClosestPoint(highway_out_, cur_xy_);

        for (auto &obs : detected_obstacles_)
        {
            // obs가 in_course 인지, out_course 인지 판단
            auto nearest_in_point = FindClosestPoint(highway_in_, obs.pose);
            auto nearest_out_point = FindClosestPoint(highway_out_, obs.pose);

            float distance_in = CalDistance(nearest_in_point, obs.pose);
            float distance_out = CalDistance(nearest_out_point, obs.pose);

            if (distance_in < 1.5)
            {
                obs.course = 1;

                ///
                //! @brief area must in [1, 2, 3]
                //!         1 : Front
                //!         2 : Middle
                //!         3 : Back
                ///                temp.object = obs;

                // initial idx가 끼어있을 경우 대처
                if (abs(nearest_cur_in.idx - nearest_in_point.idx) > in_size_ - collision_comp_)
                {
                    (nearest_cur_in.idx < nearest_in_point.idx) ? (nearest_cur_in.idx += in_size_) : (nearest_in_point.idx += in_size_);
                }

                // obstacle의 idx가 state보다 10이상 클 때, Front 위치로 판단
                if (nearest_cur_in.idx < nearest_in_point.idx - collision_comp_)
                {
                    obs.area = 1;
                }
                // obstacle의 idx가 state보다 10이상 작을 때, Back 위치로 판단
                else if (nearest_cur_in.idx > nearest_in_point.idx + collision_comp_)
                {
                    obs.area = 3;
                }
                else
                    obs.area = 2;

                // cur_state와 obs과의 거리를 speed로 나눈 시간
                // 상대속도 적용 필요
                obs.collision_time = CalDistance(cur_xy_, obs.pose) / (speed_ * 1000 / 3600);
                detected_valid_.push_back(obs);
            }

            else if (distance_out < 1.5)
            {
                obs.course = 2;

                // initial idx가 끼어있을 경우 대처
                if (abs(nearest_cur_out.idx - nearest_out_point.idx) > out_size_ - collision_comp_)
                {
                    (nearest_cur_out.idx < nearest_out_point.idx) ? (nearest_cur_out.idx += out_size_) : (nearest_out_point.idx += out_size_);
                }

                // obstacle의 idx가 state보다 10이상 클 때, Front 위치로 판단
                if (nearest_cur_out.idx < nearest_out_point.idx - collision_comp_)
                {
                    obs.area = 1;
                }
                // obstacle의 idx가 state보다 10이상 작을 때, Back 위치로 판단
                else if (nearest_cur_out.idx > nearest_out_point.idx + collision_comp_)
                {
                    obs.area = 3;
                }
                else
                    obs.area = 2;
                // cur_state와 obs과의 거리를 speed로 나눈 시간
                // 상대속도 적용 필요
                obs.collision_time = CalDistance(cur_xy_, obs.pose) / (speed_ * 1000 / 3600);
                detected_valid_.push_back(obs);
            }
        }
    }

    void CollisionSafety::CollisionCheck()
    {
        rr_common::CollisionSafety collision_check_;

        // if detected_valid is empty, is_collide = false
        collision_check_.is_collide = !detected_valid_.empty();

        // Set collision space
        collision_check_.collision_space = {false};
        int temp_space[2][3] = {0};

        for (auto &obs : detected_valid_)
        {
            temp_space[obs.course][obs.area] = 1;
        }

        int num = 0;
        for (auto &i : temp_space)
        {
            if (i)
            {
                collision_check_.collision_space[num] = true;
            }
            num++;
        }

        pub_collision_safety_.publish(collision_check_);
    }

    rr_common::PointXY CollisionSafety::CoordConvert(geometry_msgs::Pose p)
    {
        rr_common::PointXY temp;

        temp.x = p.position.x * cos((heading_ - 90) * M_PI / 180) + p.position.y * sin((heading_ - 90) * M_PI / 180) + cur_xy_.x;
        temp.y = -p.position.x * sin((heading_ - 90) * M_PI / 180) + p.position.y * cos((heading_ - 90) * M_PI / 180) + cur_xy_.y;
        temp.idx = 0;

        return temp;
    }

    rr_common::PointXY CollisionSafety::FindClosestPoint(const std::vector<rr_common::PointXY> &way_points, rr_common::PointXY p)
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

    void CollisionSafety::EmergencyStop()
    {
        //! @brief Emergency stop code here.
    }

    float CollisionSafety::CalDistance(rr_common::PointXY point1, rr_common::PointXY point2)
    {
        return sqrt(pow((point1.x - point2.x), 2) + pow((point1.y - point2.y), 2));
    }
}
