#ifndef RR_SIMUL_RR_ERP42_SERIAL_COMMAND_ERP42_SERIAL_COMMAND_H_
#define RR_SIMUL_RR_ERP42_SERIAL_COMMAND_ERP42_SERIAL_COMMAND_H_

#include <rr_common/ERP42FeedbackExt.h>
#include <rr_common/ERP42CommandRaw.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/CtrlCmd.h>
#include <ros/ros.h>

namespace rr_simul
{
    class MoraiSerialConverter
    {
    public:
        MoraiSerialConverter();
        ~MoraiSerialConverter();

        void Init(ros::NodeHandle &nh);
        ///
        //!@brief callback for feedback from morai, and cmd from 2nd stack
        ///
        void MoraiEgoCallback(const morai_msgs::EgoVehicleStatus::Ptr &ego);
        void ErpCmdCallback(const rr_common::ERP42CommandRaw::Ptr &cmd);
        void SpinOnce();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // RR_SIMUL_RR_ERP42_SERIAL_COMMAND_ERP42_SERIAL_COMMAND_H_
