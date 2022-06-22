#ifndef RR_PERCEPTION_RR_OBJECT_TRACKER_KALMAN_BOX_TRACKER_H_
#define RR_PERCEPTION_RR_OBJECT_TRACKER_KALMAN_BOX_TRACKER_H_
#include "rr_object_tracker/box_types.h"
#include "rr_object_tracker/kalman_filter.h"

#include <memory>
#include <vector>

namespace rr_perception
{

    /// convert kalman state x to OBB
    bool ConvertXToBox(const KalmanFilterState &x, OBB &box);

    /// convert kalman observation to OBB
    KalmanFilterObservation ConvertBoxToZ(const KalmanFilterState &x, const OBB &box);

    class KalmanBoxTracker
    {
    public:
        KalmanBoxTracker();
        ~KalmanBoxTracker();
        void Init(std::shared_ptr<KalmanFilter> kf, OBB &bbox);
        void Update(OBB &bbox);
        bool Predict(OBB &bbox);

        KalmanFilterState &GetState() const;
        const int &GetId() const;
        const int &GetTimeSinceUpdate() const;
        const int &GetHitStreak() const;
        bool IsValidState();

    public:
        static std::shared_ptr<KalmanFilter> CreateKalmanFilter();

    private:
    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}

#endif //RR_PERCEPTION_RR_OBJECT_TRACKER_KALMAN_BOX_TRACKER_H_