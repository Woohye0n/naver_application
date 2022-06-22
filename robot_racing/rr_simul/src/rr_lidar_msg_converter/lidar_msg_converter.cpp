#include "rr_lidar_msg_converter/lidar_msg_converter.h"
#include "velodyne_pcl/point_types.h"
#include <pcl_conversions/pcl_conversions.h>

namespace rr_simul
{

    struct MoraiLidarConverter::Impl
    {
        ///
        //! @brief publish velodyne msg to upper stack
        ///
        ros::Publisher pub_velodyne_;
        ///
        //! @brief subscribe velodyne msg from morai
        ///
        ros::Subscriber sub_velodyne_;

        ///
        //!@brief msg about velodyne data to publish
        ///
        sensor_msgs::PointCloud2 msg;
        ///
        //!@brief current velodyne ring angle range
        ///
        std::vector<float> ring_angle_bounds_;
        ///
        //!@brief compensated ring angle value calculated by past zero angle
        ///
        float prev_zero_angle_;
    };

    MoraiLidarConverter::MoraiLidarConverter() : impl_(new Impl) {}
    MoraiLidarConverter::~MoraiLidarConverter() {}

    void MoraiLidarConverter::Init(ros::NodeHandle &nh)
    {
        // ready for subscribe
        impl_->sub_velodyne_ = nh.subscribe("morai_lidar", 3, &MoraiLidarConverter::VelodyneCallback, this);

        // ready for publish
        impl_->pub_velodyne_ = nh.advertise<sensor_msgs::PointCloud2>("velodyne", 3);

        // clear the ring angle and compensate value
        impl_->ring_angle_bounds_.clear();
        impl_->prev_zero_angle_ = 0.0;
    }

    bool is_near(const float &a, const float &b)
    {
        //angel range for one ring
        return abs(a - b) < 0.015;
    }

    struct AngleBase
    {
        float base_angle_;
        float normalized(const float &ang) const
        {
            if ((ang - base_angle_) < 0)
            {
                return ang + (M_PI * 2);
            }
            else if ((ang - base_angle_) > (M_PI * 2))
            {
                return ang - (M_PI * 2);
            }
            return ang;
        }
        virtual float Angle(const velodyne_pcl::PointXYZIRT &p) const = 0;
    };

    struct VerticalAngle : public AngleBase
    {
        virtual float Angle(const velodyne_pcl::PointXYZIRT &p) const
        {
            const float &l = sqrt(p.x * p.x + p.y * p.y);
            const float &th = asin((p.z) / l);
            return normalized(th);
        }
    };
    struct HorizontalAngle : public AngleBase
    {
        virtual float Angle(const velodyne_pcl::PointXYZIRT &p) const
        {
            const float &l = sqrt(p.x * p.x + p.y * p.y);
            const float &th = atan2(p.y / l, p.x / l);
            return normalized(th);
        }
    };

    struct HorizontalAngleComparatorLess : public HorizontalAngle
    {
        bool operator()(const velodyne_pcl::PointXYZIRT &p0, const velodyne_pcl::PointXYZIRT &p1) const
        {
            return Angle(p0) < Angle(p1);
        }
    };
    struct VerticalAngleComparatorLess : public VerticalAngle
    {
        bool operator()(const velodyne_pcl::PointXYZIRT &p0, const velodyne_pcl::PointXYZIRT &p1) const
        {
            return Angle(p0) < Angle(p1);
        }
    };

    void MoraiLidarConverter::VelodyneCallback(const sensor_msgs::PointCloud2::Ptr &point)
    {
        // parse message
        pcl::PointCloud<pcl::PointXYZI> pc_in;
        pcl::PointCloud<velodyne_pcl::PointXYZIRT> pc;
        velodyne_pcl::PointXYZIRT point_pc;

        pcl::fromROSMsg(*point, pc_in);

        auto it_pc_in = pc_in.begin();
        auto end_pc_in = pc_in.end();

        while (it_pc_in != end_pc_in)
        {
            point_pc.x = it_pc_in->x;
            point_pc.y = it_pc_in->y;
            point_pc.z = it_pc_in->z;
            point_pc.intensity = it_pc_in->intensity;

            pc.push_back(point_pc);
            it_pc_in++;
        }

        // this is first message comming. make 16 ring bounds
        // if (ring_angle_bounds_.size() < 16)
        std::vector<float> vb_l(16, 0.0);
        std::vector<float> vb_u(16, 0.0);
        // make angle extractor
        std::vector<float> v_angs;

        VerticalAngle v_a;
        v_a.base_angle_ = -M_PI_2;

        // iterate over pc
        auto it = pc.begin();
        auto end = pc.end();
        for (; it != end; ++it)
        {
            v_angs.push_back(v_a.Angle(*it));
        }

        // sort vertical angles
        std::sort(v_angs.begin(), v_angs.end());

        // get min max
        float min_v_ang = v_angs.front();
        float max_v_ang = v_angs.back();

        // suggest first k means.
        impl_->ring_angle_bounds_.clear();
        for (int i = 0; i < 16; ++i)
        {
            const float &expected_mean = min_v_ang + (max_v_ang - min_v_ang) / 15.0 * i;
            impl_->ring_angle_bounds_.push_back(expected_mean);
        }

        // initialize c index
        std::vector<float> v_cidx(v_angs.size(), 0.0);
        for (int i = 0; i < v_angs.size(); ++i)
        {
            const float &cur_dist = abs(v_angs[i] - impl_->ring_angle_bounds_[v_cidx[i]]);
            for (int c = 0; c < impl_->ring_angle_bounds_.size(); ++c)
            {
                const float &new_dist = abs(v_angs[i] - impl_->ring_angle_bounds_[c]);
                v_cidx[i] = cur_dist > new_dist ? c : v_cidx[i];
            }
        }

        //k-mean clustering, k= 16
        bool changed = false;
        do
        {
            bool changed = false;
            for (int i = 0; i < v_angs.size(); ++i)
            {
                const float &cur_dist = abs(v_angs[i] - impl_->ring_angle_bounds_[v_cidx[i]]);
                for (int c = 0; c < impl_->ring_angle_bounds_.size(); ++c)
                {
                    const float &new_dist = abs(v_angs[i] - impl_->ring_angle_bounds_[c]);
                    if (cur_dist > new_dist)
                    {
                        v_cidx[i] = c;
                        changed = true;
                    }
                }
            }
            // calculate new means
            if (changed)
            {
                // set ring bounds zero.
                std::fill(impl_->ring_angle_bounds_.begin(), impl_->ring_angle_bounds_.end(), 0.0);
                std::fill(vb_l.begin(), vb_l.end(), 9999999.);
                std::fill(vb_u.begin(), vb_u.end(), -9999999.);
                std::vector<int> num_points(impl_->ring_angle_bounds_.size(), 0);
                for (int i = 0; i < v_angs.size(); ++i)
                {
                    impl_->ring_angle_bounds_[v_cidx[i]] += v_angs[i];
                    num_points[v_cidx[i]] += 1;
                    vb_l[v_cidx[i]] = vb_l[v_cidx[i]] < v_angs[i] ? vb_l[v_cidx[i]] : v_angs[i];
                    vb_u[v_cidx[i]] = vb_u[v_cidx[i]] > v_angs[i] ? vb_u[v_cidx[i]] : v_angs[i];
                }
                for (int c = 0; c < impl_->ring_angle_bounds_.size(); ++c)
                {
                    impl_->ring_angle_bounds_[c] /= num_points[c];
                }
            }
        } while (changed);

        auto it_pc = pc.begin();
        for (; it_pc != pc.end(); ++it_pc)
        {
            const float &l = sqrt(it_pc->x * it_pc->x + it_pc->y * it_pc->y);
            const float &vert_angle = asin(it_pc->z / l);
            for (int i = 0; i < impl_->ring_angle_bounds_.size(); ++i)
            {
                if (vert_angle <= vb_u[i] && vert_angle >= vb_l[i])
                {
                    it_pc->ring = i;
                    break;
                }
            }
        }

        HorizontalAngleComparatorLess comp;
        comp.base_angle_ = impl_->prev_zero_angle_;
        std::sort(pc.begin(), pc.end(), comp);

        pcl::toROSMsg(pc, impl_->msg);
        impl_->msg.header.frame_id = "velodyne";
        impl_->prev_zero_angle_ += 0.03;
    }

    void MoraiLidarConverter::SpinOnce()
    {
        impl_->pub_velodyne_.publish(impl_->msg);
    }
}