#ifndef RR_SIMUL_RR_FAKE_CAM_FAKE_CAM_H_
#define RR_SIMUL_RR_FAKE_CAM_FAKE_CAM_H_

#include <sensor_msgs/Image.h>
#include <ros/ros.h>

namespace rr_simul
{
    class MoraiCamConverter
    {
    public:
        MoraiCamConverter();
        ~MoraiCamConverter();

        void Init(ros::NodeHandle &nh);
        void SpinOnce();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // RR_SIMUL_RR_FAKE_CAM_FAKE_CAM_H_
