#include "rr_object_tracker/kalman_filter.h"

namespace rr_perception
{
    KalmanFilter::KalmanFilter() {}
    KalmanFilter::~KalmanFilter() {}
    void KalmanFilter::SetStateTransitionModel(const Eigen::MatrixXf &F, const Eigen::MatrixXf &B, const Eigen::MatrixXf &Q)
    {
        F_ = F;
        B_ = B;
        Q_ = Q;
    }

    void KalmanFilter::SetStateTransitionModelWithoutInput(const Eigen::MatrixXf &F, const Eigen::MatrixXf &Q)
    {
        F_ = F;
        Q_ = Q;
    }

    void KalmanFilter::SetObservationModel(const Eigen::MatrixXf &H, const Eigen::MatrixXf &R)
    {
        H_ = H;
        R_ = R;
    }

    void KalmanFilter::Predict(KalmanFilterState &state, const KalmanFilterControlInput &u)
    {
        state.x = F_ * state.x + B_ * u;
        state.P = F_ * state.P * F_.transpose() + Q_;
    }

    void KalmanFilter::Predict(KalmanFilterState &state)
    {
        state.x = F_ * state.x;
        state.P = F_ * state.P * F_.transpose() + Q_;
    }

    void KalmanFilter::Update(KalmanFilterState &state, const KalmanFilterObservation &z)
    {
        // innovation
        const Eigen::MatrixXf &y = z - H_ * state.x;
        // innovation covariance
        const Eigen::MatrixXf &S = H_ * state.P * H_.transpose() + R_;
        // optimal Kalman gain
        const Eigen::MatrixXf &K = state.P * H_.transpose() * S.inverse();
        // updated (a posteriori) state estimate
        state.x = state.x + K * y;
        // updated (a posteriori) state estimate covariance
        state.P = (Eigen::MatrixXf::Identity(state.P.rows(), state.P.cols()) - K * H_) * state.P;
    }
}