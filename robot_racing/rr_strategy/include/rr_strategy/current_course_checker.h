#ifndef RR_STRATEGY_RR_STRATEGY_CURRENT_COURSE_CHECKER_H_
#define RR_STRATEGY_RR_STRATEGY_CURRENT_COURSE_CHECKER_H_

#include "rr_common/PointXY.h"

#include <ros/ros.h>
#include <vector>
#include <memory>

namespace rr_strategy
{
    class CurrentCourseChecker
    {
    public:
        CurrentCourseChecker();
        ~CurrentCourseChecker();
        void Init(ros::NodeHandle &nh);
        void SetMap(const std::vector<rr_common::PointXY> &map_in,
                    const std::vector<rr_common::PointXY> &map_out);
        void SetState(const rr_common::PointXY &state);
        void FindCurrentCourse();
        bool IsOk();
        int GetCurrentCourse();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}

#endif // RR_STRATEGY_RR_STRATEGY_CURRENT_COURSE_CHECKER_H_