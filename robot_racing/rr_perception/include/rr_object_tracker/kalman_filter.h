#ifndef RR_PERCEPTION_RR_OBJECT_TRACKER_KALMAN_FILTER_H_
#define RR_PERCEPTION_RR_OBJECT_TRACKER_KALMAN_FILTER_H_
#include <Eigen/Dense>

namespace rr_perception
{
    using KalmanFilterObservation = Eigen::VectorXf;
    using KalmanFilterControlInput = Eigen::VectorXf;

    struct KalmanFilterState
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::VectorXf x;
        Eigen::MatrixXf P;
    };

    class KalmanFilter
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:
        ///
        //! @brief Construct a new Kalman Filter
        ///
        KalmanFilter();
        ///
        //! @brief Destroy the Kalman Filter
        //!
        ///
        ~KalmanFilter();
        ///
        //! @brief Set the State Transition Model: x_k+1 = F*x_k + B*u_k+ w_Q , where w_Q ~ NormalDistribution(0, Q)
        //! x_k+1 is {k+1}-th prediction (priori), x_k is k-th estimation, u is control input, and w_q is process noise.
        //!
        //! @param[in] F  State transition model F
        //! @param[in] B  State transition model B
        //! @param[in] Q  State transition model Q
        ///
        void SetStateTransitionModel(const Eigen::MatrixXf &F, const Eigen::MatrixXf &B, const Eigen::MatrixXf &Q);
        ///
        //! @brief Set the State Transition Model: x_k+1 = F*x_k + w_Q , where w_Q ~ NormalDistribution(0, Q)
        //! x_k+1 is {k+1}-th prediction (priori), x_k is k-th estimation, u is control input, and w_q is process noise.
        //!
        //! @param[in] F  State transition model F
        //! @param[in] Q  State transition model Q
        ///
        void SetStateTransitionModelWithoutInput(const Eigen::MatrixXf &F, const Eigen::MatrixXf &Q);
        ///
        //! @brief Set the Observation Model: z_k = H*x_k + w_R , where w_R ~ NormalDistribution(0, R)
        //! z is observation, x_k is k-th state, w_R is observation noise
        //!
        //! @param[in] H Observation model H
        //! @param[in] R Observation model R
        ///
        void SetObservationModel(const Eigen::MatrixXf &H, const Eigen::MatrixXf &R);
        ///
        //! @brief Kalman filter prediction with input
        //!
        //! @param[in,out] state updated state of the target. after function call, this becomes prediction
        //! @param[in] u control input for the prediction
        ///
        void Predict(KalmanFilterState &state, const KalmanFilterControlInput &u);
        ///
        //! @brief Kalman filter prediction without input
        //!
        //! @param[in,out] state updated state of the target. after function call, this becomes prediction
        ///
        void Predict(KalmanFilterState &state);
        ///
        //! @brief Kalman filter update posterior
        //!
        //! @param[in,out] state predicted state of the target. after function call, this becomes estimate.
        //! @param[in] z observation
        ///
        void Update(KalmanFilterState &state, const KalmanFilterObservation &z);

    private:
        ///
        //! @brief State transition model parameter F.Transition Model: x_k+1 = F*x_k + B*u_k+ w_Q , where w_Q ~ NormalDistribution(0, Q)
        //! x_k+1 is {k+1}-th prediction (priori), x_k is k-th estimation, u is control input, and w_q is process noise.
        ///
        Eigen::MatrixXf F_;
        ///
        //! @brief State transition model parameter B. Transition Model: x_k+1 = F*x_k + B*u_k+ w_Q , where w_Q ~ NormalDistribution(0, Q)
        //! x_k+1 is {k+1}-th prediction (priori), x_k is k-th estimation, u is control input, and w_q is process noise.
        ///
        Eigen::MatrixXf B_;
        ///
        //! @brief State transition model parameter Q. Transition Model: x_k+1 = F*x_k + B*u_k+ w_Q , where w_Q ~ NormalDistribution(0, Q)
        //! x_k+1 is {k+1}-th prediction (priori), x_k is k-th estimation, u is control input, and w_q is process noise.
        ///
        Eigen::MatrixXf Q_;
        ///
        //! @brief Observation model parameter H. Observation Model: z_k = H*x_k + w_R , where w_R ~ NormalDistribution(0, R)
        //! z is observation, x_k is k-th state, w_R is observation noise
        ///
        Eigen::MatrixXf H_;
        ///
        //! @brief Observation model parameter R. Observation Model: z_k = H*x_k + w_R , where w_R ~ NormalDistribution(0, R)
        //! z is observation, x_k is k-th state, w_R is observation noise
        ///
        Eigen::MatrixXf R_;
    };
}
#endif //RR_PERCEPTION_RR_OBJECT_TRACKER_KALMAN_FILTER_H_