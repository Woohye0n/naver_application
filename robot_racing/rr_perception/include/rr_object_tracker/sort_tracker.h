#ifndef RR_PERCEPTION_RR_OBJECT_TRACKER_SORT_TRACKER_H_
#define RR_PERCEPTION_RR_OBJECT_TRACKER_SORT_TRACKER_H_

#include "rr_object_tracker/kalman_filter.h"
#include "rr_object_tracker/kalman_box_tracker.h"
#include "rr_object_tracker/lap.h"

#include <vector>
#include <memory>

namespace rr_perception
{
    class SORTTracker
    {
    public:
        SORTTracker();
        ~SORTTracker();
        void Init(int max_age = 1, int min_hits = 3, float iou_threshold = 0.3);
        void SetDetections(const std::vector<OBB> &detections);
        std::vector<std::shared_ptr<KalmanBoxTracker>> &GetResult();
        void Update();

    private:
        void AssociateDetectionsToTrackers();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

}

#endif // RR_PERCEPTION_RR_OBJECT_TRACKER_SORT_TRACKER_H_