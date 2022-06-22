#ifndef RR_STATE_ESTIMATION_RR_STATE_ESTIMATION_RR_STATE_ESTIMATION_H_
#define RR_STATE_ESTIMATION_RR_STATE_ESTIMATION_RR_STATE_ESTIMATION_H_

#include "rr_common/LapPoint.h"
#include "rr_common/ERP42FeedbackExt.h"
#include "rr_common/StateEstimated.h"
#include "rr_common/PointXY.h"
#include "rr_common/PointGps.h"
#include <ublox_msgs/ublox_msgs.h>
#include <ros/ros.h>

#include <vector>
#include <cmath>
#include <iostream>

namespace rr_state_estimation
{
    class StateEstimation
    {
    public:
        ///
        //! @brief request service once.
        //!  service contains origin, lap check in, lap check out gps points
        ///
        StateEstimation();
        ~StateEstimation();
        ///
        //! @brief subscribe gps and erp feedback
        ///
        void Subscribe();

        ///
        //! @brief request service
        ///
        void Init(ros::NodeHandle &nh);

        ///
        //! @brief callback function for subscribe
        ///
        void GpsCallback(const ublox_msgs::NavPVT::Ptr &gps);
        void ERPCallback(const rr_common::ERP42FeedbackExt::Ptr &feedback);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;

        ///
        //! @brief convert gps to xy coordinate
        ///
        void ConvertGps2XY(const rr_common::PointGps &gps);
    };
}
#endif // RR_STATE_ESTIMATION_RR_STATE_ESTIMATION_RR_STATE_ESTIMATION_H_
