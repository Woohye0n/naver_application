#include "rr_object_tracker/kalman_box_tracker.h"
#include "rr_object_tracker/kalman_filter.h"
#include "rr_object_tracker/polygon.h"
#include <iostream>
namespace rr_perception
{

    bool ConvertXToBox(const KalmanFilterState &x, OBB &box)
    {
        Eigen::Vector2f c = x.x.block(0, 0, 2, 1);
        const float &th = x.x(2);
        const float &s = x.x(3);
        const float &r = x.x(4);

        if (s < 0 || r < 0)
        {
            return false;
        }

        const float &w = std::sqrt(s * r);
        const float &h = s / w;

        Eigen::Matrix2f rot;
        rot << cos(th), -sin(th),
            sin(th), cos(th);
        const float &w_2 = w * 0.5;
        const float &h_2 = h * 0.5;

        // pivot, p0, p1
        Eigen::MatrixXf pp;
        pp.resize(2, 3);
        pp << w_2, -w_2, w_2,
            h_2, h_2, -h_2;

        Eigen::MatrixXf points = rot * pp;
        points.colwise() += c;

        box.p_pivot = points.block(0, 0, 2, 1);
        box.p0 = points.block(0, 1, 2, 1);
        box.p1 = points.block(0, 2, 2, 1);
        return true;
    }

    KalmanFilterObservation ConvertBoxToZ(const KalmanFilterState &x, const OBB &box)
    {
        KalmanFilterObservation ob;
        ob.resize(5);
        ob.block(0, 0, 2, 1) = GetCenter(box);

        // const float &th = x.x(2);
        const float &s = x.x(3);
        const float &r = x.x(4);

        // override
        const Eigen::Vector2f &vec_w = box.p0 - box.p_pivot;
        const Eigen::Vector2f &vec_h = box.p1 - box.p_pivot;
        ob(2) = atan2(vec_w(1), vec_w(0));
        const float &w = vec_w.norm();
        const float &h = vec_h.norm();
        ob(3) = w * h;
        ob(4) = w / h;
        return ob;

        // s or r is 0. this is invalid state.
        if (s <= 0 || r <= 0)
        {
            const Eigen::Vector2f &vec_w = box.p0 - box.p_pivot;
            const Eigen::Vector2f &vec_h = box.p1 - box.p_pivot;
            ob(2) = atan2(vec_w(1), vec_w(0));
            const float &w = vec_w.norm();
            const float &h = vec_h.norm();
            ob(3) = w * h;
            ob(4) = w / h;
        }
        else
        {
            // this state x is valid. find theta.
            const Eigen::Vector2f &c = GetCenter(box);
            const Eigen::Vector2f &vec_w = box.p0 - box.p_pivot;
            const Eigen::Vector2f &vec_h = box.p1 - box.p_pivot;

            Polygon axis(4);
            axis.push_back(vec_w.normalized());
            axis.push_back(vec_h.normalized());
            axis.push_back(-axis[0]);
            axis.push_back(-axis[1]);

            // find the theta in x in the range [-pi , pi]
            const float &th = x.x(2);
            const float &c_th = cos(th);
            const float &s_th = sin(th);

            // calculate abs angle difference and find min diff and min index

            float diff = atan2(axis[0](0) * s_th - axis[0](1) * c_th,
                               axis[0](0) * c_th + axis[0](1) * s_th);
            float min_abs_diff = std::fabs(diff);
            int min_idx = 0;
            for (int i = 1; i < 4; ++i)
            {
                const float &c_diff = axis[i](0) * c_th + axis[i](1) * s_th;
                const float &s_diff = axis[i](0) * s_th - axis[i](1) * c_th;
                const float &th_diff = atan2(s_diff, c_diff);
                const float &th_abs_diff = std::fabs(th_diff);
                if (th_abs_diff < min_abs_diff)
                {
                    min_abs_diff = th_abs_diff;
                    diff = th_diff;
                    min_idx = i;
                }
            }

            // generate closest box theta
            ob(2) = th + diff;

            // choose w and h
            const float &w = (min_idx % 2 == 0) ? vec_w.norm() : vec_h.norm();
            const float &h = (min_idx + 1 % 2 == 0) ? vec_h.norm() : vec_w.norm();

            // set s and r
            ob(3) = w * h;
            ob(4) = w / h;
        }
        return ob;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////
    //
    //                               Kalman Box Tracker Definitions
    //
    //           x is
    //           [0]: center x
    //           [1]: center y
    //           [2]: th = angle to the w axis
    //           [3]: s = w*h
    //           [4]: r = w/h
    //           [5]: dx
    //           [6]: dy
    //           [7]: dth
    //           [8]: ds
    //
    //           z is
    //           [0]: center x
    //           [1]: center y
    //           [2]: th = angle to the w axis
    //           [3]: s = w*h
    //           [4]: r = w/h
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////

    struct KalmanBoxTracker::Impl
    {
        std::shared_ptr<KalmanFilter> kf;
        KalmanFilterState kf_state;

        static int count;
        int time_since_update;
        int hits;
        int hit_streak;
        int id;
        int age;
    };
    int KalmanBoxTracker::Impl::count = 0;

    KalmanBoxTracker::KalmanBoxTracker() : impl_(new Impl) {}

    KalmanBoxTracker::~KalmanBoxTracker() {}

    void KalmanBoxTracker::Init(std::shared_ptr<KalmanFilter> kf, OBB &bbox)
    {
        impl_->kf = kf;
        const int &x_dim = 9;
        const int &z_dim = 5;

        // state transition model
        // [0]: center x
        // [1]: center y
        // [2]: th = angle to the w axis
        // [3]: s = w*h
        // [4]: r = w/h
        // [5]: dx
        // [6]: dy
        // [7]: dth
        // [8]: ds

        impl_->kf_state.P.setIdentity(x_dim, x_dim);
        impl_->kf_state.P.block(5, 5, 4, 4) *= 1000.0;
        impl_->kf_state.P *= 10.0;

        impl_->kf_state.x.setZero(x_dim);
        impl_->kf_state.x.block(0, 0, 5, 1) = ConvertBoxToZ(impl_->kf_state, bbox);

        impl_->time_since_update = 0;

        impl_->id = impl_->count;
        impl_->count += 1;
        impl_->hits = 0;
        impl_->hit_streak = 0;
        impl_->age = 0;
    }
    void KalmanBoxTracker::Update(OBB &bbox)
    {
        // Updates the state vector with observed bbox.
        impl_->time_since_update = 0;
        impl_->hits += 1;
        impl_->hit_streak += 1;

        // Kalman filter update
        KalmanFilterObservation observation = ConvertBoxToZ(impl_->kf_state, bbox);
        impl_->kf->Update(impl_->kf_state, observation);
    }

    bool KalmanBoxTracker::Predict(OBB &bbox)
    {
        // Advances the state vector and returns the predicted bounding box estimate.
        // small or invisible boxes are not going to change their size.

        // state correction
        // state transition model
        // [0]: center x
        // [1]: center y
        // [2]: th = angle to the w axis
        // [3]: s = w*h
        // [4]: r = w/h
        // [5]: dx
        // [6]: dy
        // [7]: dth
        // [8]: ds
        if (impl_->kf_state.x(8) + impl_->kf_state.x(3) <= 0)
        {
            impl_->kf_state.x(8) *= 0.0;
        }

        impl_->kf_state.x(2) = fmod(impl_->kf_state.x(2), 2 * M_PI);

        impl_->kf->Predict(impl_->kf_state);
        if (!IsValidState())
        {
            return false;
        }

        impl_->age += 1;
        if (impl_->time_since_update > 0)
        {
            impl_->hit_streak = 0;
        }
        impl_->time_since_update += 1;
        bool ret = ConvertXToBox(impl_->kf_state, bbox);
        return ret;
    }

    KalmanFilterState &KalmanBoxTracker::GetState() const { return impl_->kf_state; }

    const int &KalmanBoxTracker::GetId() const { return impl_->id; }

    const int &KalmanBoxTracker::GetTimeSinceUpdate() const { return impl_->time_since_update; }

    const int &KalmanBoxTracker::GetHitStreak() const { return impl_->hit_streak; }

    std::shared_ptr<KalmanFilter> KalmanBoxTracker::CreateKalmanFilter()
    {
        std::shared_ptr<KalmanFilter> kf(new KalmanFilter);
        const int &x_dim = 9;
        const int &z_dim = 5;

        Eigen::MatrixXf F(x_dim, x_dim); // state transition model
        F << 1, 0, 0, 0, 0, 1, 0, 0, 0,  // [0]: center x
            0, 1, 0, 0, 0, 0, 1, 0, 0,   // [1]: center y
            0, 0, 1, 0, 0, 0, 0, 1, 0,   // [2]: th = angle to the w axis
            0, 0, 0, 1, 0, 0, 0, 0, 1,   // [3]: s = w*h
            0, 0, 0, 0, 1, 0, 0, 0, 0,   // [4]: r = w/h
            0, 0, 0, 0, 0, 1, 0, 0, 0,   // [5]: dx
            0, 0, 0, 0, 0, 0, 1, 0, 0,   // [6]: dy
            0, 0, 0, 0, 0, 0, 0, 1, 0,   // [7]: dth
            0, 0, 0, 0, 0, 0, 0, 0, 1;   // [8]: ds

        Eigen::VectorXf Q_diag; // state transition model
        Q_diag.resize(x_dim);   //
        Q_diag(0) = 0.05;       // [0]: center x     0.05m
        Q_diag(1) = 0.05;       // [1]: center y     0.05m
        Q_diag(2) = 0.2;        // [2]: th = angle to the w axis 0.05 radian ~ 1.57/8
        Q_diag(3) = 0.00001;    // [3]: s = w*h      1m^2
        Q_diag(4) = 0.2;        // [4]: r = w/h      0.2 ratio
        Q_diag(5) = 0.1;        // [5]: dx           0.5 m/s
        Q_diag(6) = 0.1;        // [6]: dy           0.5 m/s
        Q_diag(7) = 0.01;       // [7]: dth          0.1 rad/s
        Q_diag(8) = 0.001;      // [8]: ds           0.001 m^2/s

        Eigen::MatrixXf H(z_dim, x_dim); // observation model
        H << 1, 0, 0, 0, 0, 0, 0, 0, 0,  // [0]: center x
            0, 1, 0, 0, 0, 0, 0, 0, 0,   // [1]: center y
            0, 0, 1, 0, 0, 0, 0, 0, 0,   // [2]: th = angle to the w axis
            0, 0, 0, 1, 0, 0, 0, 0, 0,   // [3]: s = w*h
            0, 0, 0, 0, 1, 0, 0, 0, 0;   // [4]: r = w/h

        Eigen::VectorXf R_diag; //observation model
        R_diag.resize(z_dim);   //
        R_diag(0) = 0.5;        // [0]: center x      0.5 m
        R_diag(1) = 0.5;        // [1]: center y      0.5 m
        R_diag(2) = 0.1;        // [2]: th = angle to the w axis
        R_diag(3) = 3;          // [3]: s = w*h
        R_diag(4) = 0.0001;     // [4]: r = w/h

        kf->SetStateTransitionModelWithoutInput(F, Q_diag.asDiagonal());
        kf->SetObservationModel(H, R_diag.asDiagonal());
        return kf;
    }

    bool KalmanBoxTracker::IsValidState()
    {
        bool is_nan = impl_->kf_state.x.array().isNaN().any();
        bool is_inf = impl_->kf_state.x.array().isInf().any();
        return !(is_nan || is_inf);
    }
}
