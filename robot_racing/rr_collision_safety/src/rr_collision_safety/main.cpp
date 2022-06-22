#include "rr_collision_safety/rr_collision_safety.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_safety");
    rr_collision_safety::CollisionSafety collision_safety;

    ros::spin();

    return 0;
}