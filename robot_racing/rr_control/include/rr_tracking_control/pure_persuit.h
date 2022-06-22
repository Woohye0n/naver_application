#ifndef RR_CONTROL_RR_TRACKING_CONTROL_PURE_PERSUIT_H_
#define RR_CONTROL_RR_TRACKING_CONTROL_PURE_PERSUIT_H_

#include "rr_common/StateEstimated.h"
#include "rr_common/LocalPathPoints.h"
#include "rr_common/SteerControlCommand.h"
#include "rr_common/PointXY.h"
#include <ros/ros.h>
#include <memory>

namespace rr_control
{
    class PurePersuitController
    {
    public:
        PurePersuitController();
        ~PurePersuitController();
        void Init(ros::NodeHandle &nh);
        void FeedbackCallback(const rr_common::StateEstimated::Ptr &msg);
        void PathCallback(const rr_common::LocalPathPoints::Ptr &path);

    private:
        void UpdateLD();
        void UpdateWL();
        void UpdateIndex();
        void UpdateDelta();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // RR_CONTROL_RR_TRACKING_CONTROL_RR_PURE_PERSUIT_H_
