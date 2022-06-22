#include "rr_obstacle_msg_converter/obstacle_msg_converter.h"

namespace rr_simul
{
    struct MoraiObstacleConverter::Impl
    {
        ///
        //! @brief subscribe car status(heading) msg from morai
        ///
        ros::Subscriber sub_ego;
        ///
        //! @brief subscribe obstacles msg from morai
        ///
        ros::Subscriber sub_obs;
        ///
        //! @brief publish obstacle msg
        ///
        ros::Publisher pub_perception;

        ///
        //!@brief current car position
        ///
        geometry_msgs::Vector3 car_position;
        ///
        //!@brief obstacle data array msg to publish
        ///
        rr_common::PerceptionObstacleArray obs_array;

        //parameter to check change of time
        ros::Time last_update_timestamp;
        //parameter to check change of obstacle heading
        std::vector<float> obj_heading_array;

        // current car heading
        double heading;
        // past car heading
        double prev_heading;
        // current car velocity
        float car_velocity_x;
        // time chage about obstacle
        float dt;
        // heading change of obstacle
        float d_theta;
        // parameter for checking obstacle number changed
        int obs_num = 0;
    };

    MoraiObstacleConverter::MoraiObstacleConverter() : impl_(new Impl) {}
    MoraiObstacleConverter::~MoraiObstacleConverter() {}

    void MoraiObstacleConverter::Init(ros::NodeHandle &nh)
    {
        // ready for publish
        impl_->pub_perception = nh.advertise<rr_common::PerceptionObstacleArray>("perception", 3);

        // ready for subscribe
        impl_->sub_ego = nh.subscribe("morai_ego", 1, &MoraiObstacleConverter::MoraiEgoCallback, this);
        impl_->sub_obs = nh.subscribe("morai_obstacle", 1, &MoraiObstacleConverter::MoraiObsCallback, this);
    }

    void MoraiObstacleConverter::MoraiEgoCallback(const morai_msgs::EgoVehicleStatus::Ptr &ego)
    {
        // get heading
        impl_->heading = 90 - ego->heading;

        // get car position
        impl_->car_position = ego->position;

        // get car velocity
        impl_->car_velocity_x = ego->velocity.x;
    }

    void MoraiObstacleConverter::MoraiObsCallback(const morai_msgs::ObjectStatusList::Ptr &obs)
    {
        // get only obstacle data
        auto it_obs_in = obs->npc_list;
        rr_common::PerceptionObstacle obs_data_;

        //check obstacle num changed
        bool is_obs_changed = false;

        // check obstacle number
        if (impl_->obs_num != it_obs_in.size())
        {
            is_obs_changed = true;
            impl_->obs_num = it_obs_in.size();
        }

        //calculat delta time
        impl_->dt = float((ros::Time::now() - impl_->last_update_timestamp).toSec());
        impl_->last_update_timestamp = ros::Time::now();

        //setting header
        obs_data_.header.stamp = ros::Time::now();
        obs_data_.header.frame_id = "velodyne";
        shape_msgs::SolidPrimitive shape_;
        shape_.type = 1;

        // get heading angle difference between car and obtacle
        impl_->d_theta = impl_->heading - impl_->prev_heading;
        impl_->prev_heading = impl_->heading;

        // angular velocity
        float w_ = float(impl_->d_theta / impl_->dt) * M_PI / 180;

        impl_->obs_array.obstacles.clear();

        for (int i = 0; i < it_obs_in.size(); i++)
        {
            auto data_ = it_obs_in[i];
            // set dimension
            obs_data_.id = data_.unique_id;
            shape_.dimensions.clear();
            shape_.dimensions.push_back(data_.size.x);
            shape_.dimensions.push_back(data_.size.y);
            shape_.dimensions.push_back(data_.size.z);
            obs_data_.shape = shape_;

            float obj_heading_ = 90 - data_.heading;
            // calculate global of difference in x,y
            float x_lengh_ = data_.position.x - impl_->car_position.x;
            float y_lengh_ = data_.position.y - impl_->car_position.y;
            float total_lengh = sqrt(x_lengh_ * x_lengh_ + y_lengh_ * y_lengh_);

            float object_angle_ = (impl_->heading + atan2(y_lengh_, x_lengh_) * 180 / M_PI);

            // calculate obstacle position
            obs_data_.pose.position.x = total_lengh * cos(object_angle_ * M_PI / 180);
            obs_data_.pose.position.y = total_lengh * sin(object_angle_ * M_PI / 180);
            obs_data_.pose.position.z = float(data_.position.z - impl_->car_position.z);

            // calculate obstacle orientation
            obs_data_.pose.orientation.x = 0;
            obs_data_.pose.orientation.y = 0;
            obs_data_.pose.orientation.z = (obj_heading_ - impl_->heading) * M_PI / 180;
            obs_data_.pose.orientation.w = 1;

            // setting initial heading of obstacle if obstacle number changed
            if (is_obs_changed)
            {
                impl_->obj_heading_array.push_back(obj_heading_ - impl_->heading);
            }

            //calculate linear velocity
            obs_data_.twist.linear.x = obs_data_.pose.position.y * w_ + data_.velocity.x * sin(impl_->obj_heading_array[i] * M_PI / 180);
            obs_data_.twist.linear.y = -obs_data_.pose.position.x * w_ - impl_->car_velocity_x * 3600 / 1000 + data_.velocity.x * cos(impl_->obj_heading_array[i] * M_PI / 180);

            impl_->obs_array.obstacles.push_back(obs_data_);
        }
    }

    void MoraiObstacleConverter::SpinOnce()
    {
        // publish obstacle data in array
        impl_->pub_perception.publish(impl_->obs_array);
    }
}