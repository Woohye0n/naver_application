#include "rr_object_tracker/sort_tracker.h"

#include <vector>
#include <memory>
#include <iostream>

namespace rr_perception
{
    using Detection = KalmanFilterObservation;
    using Detections = std::vector<Detection>;

    struct SORTTracker::Impl
    {
        // tracking
        int frame_count;
        std::vector<std::shared_ptr<KalmanBoxTracker>> trackers;
        std::vector<std::shared_ptr<KalmanBoxTracker>> tracking_result;

        // configuration
        int max_age;
        int min_hits;
        float iou_threshold;

        // association
        std::vector<OBB> detections;
        std::vector<OBB> tracker_preds;
        std::vector<IndicesPair> matches;
        std::vector<int> unmatched_detections;
        std::vector<int> unmatched_trackers;

        // kalman filter
        std::shared_ptr<KalmanFilter> kf;
    };
    SORTTracker::SORTTracker() : impl_(new Impl) {}
    SORTTracker::~SORTTracker() {}
    void SORTTracker::Init(int max_age, int min_hits, float iou_threshold)
    {
        impl_->max_age = max_age;
        impl_->min_hits = min_hits;
        impl_->iou_threshold = iou_threshold;
        impl_->frame_count = 0;
        impl_->kf = KalmanBoxTracker::CreateKalmanFilter();
        // Detections dets;
        // dets.erase(std::remove_if(dets.begin(), dets.end(), comp), dets.end());
    }
    void SORTTracker::SetDetections(const std::vector<OBB> &detections) { impl_->detections = detections; }
    std::vector<std::shared_ptr<KalmanBoxTracker>> &SORTTracker::GetResult()
    {
        return impl_->tracking_result;
    }

    void SORTTracker::Update()
    {
        impl_->frame_count += 1;

        // collect valid tracker predictions.
        // delete trackers who made invalid prediction.
        impl_->tracker_preds.clear();
        auto it = impl_->trackers.begin();
        while (it != impl_->trackers.end())
        {
            OBB pred;
            bool valid = (*it)->Predict(pred);
            if (valid)
            {
                impl_->tracker_preds.push_back(pred);
                it++;
            }
            else
            {
                it = impl_->trackers.erase(it);
            }
        }

        // associate detections to trackers.
        // impl_->matches, impl_->unmatched_detections, impl_->unmatched_trackers will be updated.
        AssociateDetectionsToTrackers();

        // update matched trackers with assigned detections
        // remember. row is detections(test), col is trackers(ground truth)
        for (auto &m : impl_->matches)
        {
            impl_->trackers[m.col]->Update(impl_->detections[m.row]);
        }

        // create and initialise new trackers for unmatched detections
        for (auto &idx : impl_->unmatched_detections)
        {
            impl_->trackers.push_back(std::make_shared<KalmanBoxTracker>());
            impl_->trackers.back()->Init(impl_->kf, impl_->detections[idx]);
        }

        impl_->tracking_result.clear();
        auto itt = impl_->trackers.begin();
        for (; itt != impl_->trackers.end();)
        {
            if ((*itt)->GetTimeSinceUpdate() < 1 &&
                (*itt)->IsValidState() &&
                (((*itt)->GetHitStreak() >= impl_->min_hits) || impl_->frame_count <= impl_->min_hits))
            {
                impl_->tracking_result.push_back(*itt);
            }

            if ((*itt)->GetTimeSinceUpdate() > impl_->max_age)
            {
                itt = impl_->trackers.erase(itt);
            }
            else
            {
                itt++;
            }
        }
    }

    void SORTTracker::AssociateDetectionsToTrackers()
    {
        // initialize
        impl_->matches.clear();
        impl_->unmatched_trackers.clear();
        impl_->unmatched_detections.clear();

        // no trackers exists.
        if (impl_->tracker_preds.size() == 0)
        {
            impl_->unmatched_detections.resize(impl_->detections.size());
            for (int i = 0; i < impl_->detections.size(); ++i)
            {
                impl_->unmatched_detections[i] = i;
            }
            return;
        }

        // get the iou matrix. row is detections(test), col is trackers(ground truth)
        Eigen::MatrixXf distance_mat = DistanceBatch(impl_->detections, impl_->tracker_preds);
        const int &rows = distance_mat.rows();
        const int &cols = distance_mat.cols();

        std::vector<IndicesPair> matched_indices;
        if (rows == 0 || cols == 0)
        {
            // there is no match.
            matched_indices.clear();
        }
        else
        {
            // match exists.
            // there are multiple match for one index.
            // solve the Linear Assignment Problem.
            LAPJVAlgorithm sol;
            sol.Solve(distance_mat);
            matched_indices = sol.Solution();
        }

        // Drop the matches with low iou.
        std::vector<bool> is_match_detection(impl_->detections.size(), false);
        std::vector<bool> is_match_tracker(impl_->tracker_preds.size(), false);
        // mark only valid matches
        for (auto &m : matched_indices)
        {
            if (distance_mat(m.row, m.col) < impl_->iou_threshold)
            {
                impl_->matches.push_back({m.row, m.col});
                is_match_detection[m.row] = true;
                is_match_tracker[m.col] = true;
            }
        }
        // collect invalid matches.
        for (int i = 0; i < is_match_detection.size(); ++i)
        {
            if (!is_match_detection[i])
            {
                impl_->unmatched_detections.push_back(i);
            }
        }
        for (int i = 0; i < is_match_tracker.size(); ++i)
        {
            if (!is_match_tracker[i])
            {
                impl_->unmatched_trackers.push_back(i);
            }
        }
    }

} // namespace rr_perception
