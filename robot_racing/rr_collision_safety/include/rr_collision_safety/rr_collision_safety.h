#include "rr_common/MapWayPointXY.h"
#include "rr_common/StateEstimated.h"
#include "rr_common/LapPoint.h"
#include "rr_common/CollisionSafety.h"
#include "rr_common/PerceptionObstacle.h"
#include "rr_common/PerceptionObstacleArray.h"
#include "rr_common/PointXY.h"

#include <iostream>
#include <ros/ros.h>
#include <vector>

namespace rr_collision_safety
{
    // struct Obstacle
    // {
    //     rr_common::PerceptionObstacle object;

    //     // course must be in [1, 2]
    //     // 1 : in_course, 2 : out_course
    //     int course;

    //     // area must be in [1, 2, 3]
    //     // 1 : Front, 2 : Middle, 3 : Back (about current state)
    //     int area;

    //     // expected time to collision
    //     float collision_time;
    // };
    struct Obstacle
    {
        // course must be in [1, 2]
        // 1 : in_course, 2 : out_course
        int course;

        // area must be in [1, 2, 3]
        // 1 : Front, 2 : Middle, 3 : Back (about current state)
        int area;

        // expected time to collision
        float collision_time;

        int id;

        // obstacle shape
        shape_msgs::SolidPrimitive shape;

        // obstacle pose
        rr_common::PointXY pose;

        // obstacle velocity
        geometry_msgs::Twist twist;
    };

    class CollisionSafety
    {
    public:
        CollisionSafety();

        ///
        //! @brief Subscriber
        ///
        void Subscribe();

    private:
        ///
        //! @brief client to get map xy way points from rr_map
        //! subscriber to get current state from rr_state_estimation
        //! publisher collision time to rr_strategy
        ///
        ros::ServiceClient map_client_;
        ros::NodeHandle nh_;
        ros::Subscriber sub_state_;
        ros::Subscriber sub_obs_;
        ros::Publisher pub_collision_safety_;

        // racing parameters
        std::vector<rr_common::PointXY> racing_;
        std::vector<rr_common::PointXY> highway_in_;
        std::vector<rr_common::PointXY> highway_out_;

        int in_size_;
        int out_size_;

        // param
        int collision_comp_;

        // current state
        rr_common::PointXY cur_xy_;
        float heading_;
        float speed_;

        // Vector of detected obstacles
        std::vector<Obstacle> detected_obstacles_;
        std::vector<Obstacle> detected_valid_;

        // Msg callbacks

        ///
        //! @brief Callback of rr_state_estimation
        ///
        void StateCb(const rr_common::StateEstimated::Ptr &state_sub_);

        ///
        //! @brief Callback of rr_obstacle_detection
        ///
        void ObstacleCb(const rr_common::PerceptionObstacleArray::Ptr &obstacles);

        // functions

        ///
        //! @brief Receive xy coordinate map from rr_map
        ///
        void GetMapXY();

        ///
        //! @brief Collision Checker
        ///
        void CollisionCheck();

        ///
        //! @brief Figure obstacles in courses
        ///
        void FigureValidObs();

        rr_common::PointXY CoordConvert(geometry_msgs::Pose p);

        ///
        //! @brief Emergency stop
        ///
        void EmergencyStop();

        ///
        //! @brief Calculate distance between p1 and p2
        ///
        float CalDistance(rr_common::PointXY point1, rr_common::PointXY point2);

        ///
        //! @brief Find closest point in way_points from input_point
        ///
        rr_common::PointXY FindClosestPoint(const std::vector<rr_common::PointXY> &way_points, rr_common::PointXY p);
    };
}