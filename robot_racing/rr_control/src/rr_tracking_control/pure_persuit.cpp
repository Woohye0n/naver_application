#include "rr_tracking_control/pure_persuit.h"

namespace rr_control
{
    float Distance(const rr_common::PointXY &p0, const rr_common::PointXY &p1)
    {
        return sqrt(pow((p0.x - p1.x), 2) + pow((p0.y - p1.y), 2));
    }

    struct PurePersuitController::Impl
    {
        // ros
        ros::Publisher pub_steer;
        ros::Subscriber sub_state;
        ros::Subscriber sub_path;

        // latest state
        rr_common::StateEstimated latest_state;

        // path and indices
        std::vector<rr_common::PointXY> local_path;
        int idx_n1 = 0;
        int idx_n2 = 1;

        // vehicle property
        float delta_to_command;
        float L;

        // control parameter
        float arrive_distance;

        float LD;
        float delta;
        rr_common::PointXY WL_xy;
    };

    PurePersuitController::PurePersuitController() : impl_(new Impl) {}
    PurePersuitController::~PurePersuitController() {}

    void PurePersuitController::Init(ros::NodeHandle &nh)
    {
        // get parameter
        ROS_ASSERT(nh.getParam("arrive_dis", impl_->arrive_distance)); //도착 판단 거리

        // ready for publish
        impl_->pub_steer = nh.advertise<rr_common::SteerControlCommand>("steer_command", 3);

        // subscriber
        impl_->sub_path = nh.subscribe("local_path", 1, &PurePersuitController::PathCallback, this);
        impl_->sub_state = nh.subscribe("state", 1, &PurePersuitController::FeedbackCallback, this);

        // clear local path
        impl_->local_path.clear();

        // hardcoded vehicle property. change to the parameter if needed.
        impl_->delta_to_command = 102;
        impl_->L = 1.212;
    }
    void PurePersuitController::FeedbackCallback(const rr_common::StateEstimated::Ptr &msg)
    {
        // udpate latest state
        impl_->latest_state = *msg;

        // here you make control loop
        rr_common::SteerControlCommand msg_command;
        msg_command.header.frame_id = "pure_persuit";
        msg_command.header.stamp = ros::Time::now();

        // steer 0 when you don't have path.
        if (impl_->local_path.empty())
        {
            msg_command.steer = 0.0;
            impl_->pub_steer.publish(msg_command);
            return;
        }

        const rr_common::PointXY &n1 = impl_->local_path[impl_->idx_n1];
        const rr_common::PointXY &n2 = impl_->local_path[impl_->idx_n2];
        UpdateLD();
        UpdateWL();
        UpdateIndex();
        UpdateDelta();

        msg_command.steer = impl_->delta;
        impl_->pub_steer.publish(msg_command);
    }
    void PurePersuitController::PathCallback(const rr_common::LocalPathPoints::Ptr &path)
    {
        impl_->local_path.clear();
        impl_->local_path = path->local_path_points;
    }

    void PurePersuitController::UpdateLD()
    {
        impl_->LD = 4.5 + 0.1 * impl_->latest_state.speed;
        ROS_DEBUG("LD %f", impl_->LD);
    }

    void PurePersuitController::UpdateWL()
    {
        const rr_common::PointXY &n1 = impl_->local_path[impl_->idx_n1];
        const rr_common::PointXY &n2 = impl_->local_path[impl_->idx_n2];

        const float &cur_x = impl_->latest_state.current_xy.x;
        const float &cur_y = impl_->latest_state.current_xy.y;

        // 이차방정식 근의공식으로 두개의 교점 찾기
        float slope = (n2.y - n1.y) / (n2.x - n1.x);
        float a = 1 + pow(slope, 2);

        const float &b0 = -2 * cur_x;
        const float &b1 = 2 * pow(slope, 2) * n1.x;
        const float &b2 = 2 * slope * (n1.y - cur_y);
        float b = b0 - b1 + b2;

        const float &c0 = pow(cur_x, 2);
        const float &c1 = pow(slope, 2) * pow(n1.x, 2);
        const float &c2 = 2 * slope * (n1.y - cur_y) * n1.x;
        const float &c3 = pow(n1.y, 2) - 2 * n1.y * cur_y;
        const float &c4 = pow(cur_y, 2) - pow(impl_->LD, 2);
        float c = c0 + c1 - c2 + c3 + c4;

        // 두 교점 선언 및 정의
        rr_common::PointXY intersection_point1, intersection_point2;

        intersection_point1.x = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
        intersection_point2.x = (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
        intersection_point1.y = slope * (intersection_point1.x - n1.x) + n1.y;
        intersection_point2.y = slope * (intersection_point2.x - n2.x) + n2.y;

        // 두개의 교점중, 목표지점과 가까운 점을 WL_xy_에 저장
        const float &distance1 = Distance(n2, intersection_point1);
        const float &distance2 = Distance(n2, intersection_point2);

        impl_->WL_xy = (distance1 > distance2) ? intersection_point2 : intersection_point1;
    }

    void PurePersuitController::UpdateIndex()
    {
        const float &cur_x = impl_->latest_state.current_xy.x;
        const float &cur_y = impl_->latest_state.current_xy.y;
        rr_common::PointXY cur_pos;
        cur_pos.x = cur_x;
        cur_pos.y = cur_y;
        cur_pos.idx = 0;

        const rr_common::PointXY &n2 = impl_->local_path[impl_->idx_n2];

        // 로봇이 way point를 지나면 idx가 1증가
        float distance = Distance(cur_pos, n2);
        ROS_DEBUG("distance to n2 %f", distance);

        // Arrival distance와 비교
        if (distance < impl_->arrive_distance)
        {
            impl_->idx_n1 += ceil(2 * (impl_->arrive_distance - distance)) + 1;
            impl_->idx_n2 = impl_->idx_n1 + 1;
        }

        ROS_DEBUG("Abs index %d", impl_->idx_n1);
        // ROS_DEBUG("lap %d", impl_->lap_);
    }
    void PurePersuitController::UpdateDelta()
    {
        // finding alpha
        const float &X = impl_->WL_xy.x - impl_->latest_state.current_xy.x;
        const float &Y = impl_->WL_xy.y - impl_->latest_state.current_xy.y;
        float path_heading = atan2(X, Y) * (180 / M_PI);
        float alpha = path_heading - impl_->latest_state.heading; // +면 오른쪽 -면 왼쪽

        // finding delta
        float e = impl_->LD * sin(fabs(alpha) * M_PI / 180);
        impl_->delta = atan((2 * impl_->L * e) / (impl_->LD * impl_->LD)) * 180 / M_PI;
        if (alpha < 0)
        {
            impl_->delta *= -1;
        }

        ROS_DEBUG("delta %f", impl_->delta);
        ROS_DEBUG("------------------------------------------------");
    }

}