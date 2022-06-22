#include "rr_strategy/current_course_checker.h"

namespace rr_strategy
{
    struct CurrentCourseChecker::Impl
    {
        std::vector<rr_common::PointXY> map_in;
        std::vector<rr_common::PointXY> map_out;

        std::vector<rr_common::PointXY> map_in_xy;
        std::vector<rr_common::PointXY> map_out_xy;

        rr_common::PointXY position;

        int current_course;
        bool position_set = false;
    };

    CurrentCourseChecker::CurrentCourseChecker() : impl_(new Impl) {}
    CurrentCourseChecker::~CurrentCourseChecker() {}

    void CurrentCourseChecker::Init(ros::NodeHandle &nh)
    {
    }

    bool CurrentCourseChecker::IsOk()
    {
        if (impl_->position_set)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void CurrentCourseChecker::SetMap(const std::vector<rr_common::PointXY> &map_in,
                                      const std::vector<rr_common::PointXY> &map_out)
    {
        impl_->map_in = map_in;
        impl_->map_out = map_out;
    }

    void CurrentCourseChecker::SetState(const rr_common::PointXY &state)
    {
        impl_->position = state;
        impl_->position_set = true;
    }

    void CurrentCourseChecker::FindCurrentCourse()
    {
        float min_in = 9999;
        for (auto &map : impl_->map_in_xy)
        {
            float dis_in = sqrt(pow((map.x - impl_->position.x), 2) + pow((map.y - impl_->position.y), 2));
            if (dis_in < min_in)
            {
                min_in = dis_in;
            }
        }

        float min_out = 99999;
        for (auto &map : impl_->map_out_xy)
        {
            float dis_out = sqrt(pow((map.x - impl_->position.x), 2) + pow((map.y - impl_->position.y), 2));
            if (dis_out < min_out)
            {
                min_out = dis_out;
            }
        }

        impl_->current_course = (min_in >= min_out) ? 2 : 1;

        // if (!find)
        // {
        //     // still changing
        //     impl_->current_course = 0;
        // }
    }

    int CurrentCourseChecker::GetCurrentCourse()
    {
        return impl_->current_course;
    }

}