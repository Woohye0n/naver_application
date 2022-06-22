#ifndef RR_DEVICES_RR_ERP42_ERP42_H_
#define RR_DEVICES_RR_ERP42_ERP42_H_

#include "rr_common/ERP42CommandRaw.h"

#include <ros/ros.h>
#include <memory>

namespace rr_devices
{
    ///
    //! @brief ERP42 serial communication node
    ///
    class ERP42
    {
    public:
        ///
        //! @brief Create a new ERP42 node
        //!
        ///
        ERP42();
        ///
        //! @brief Destroy the ERP42 node
        ///
        ~ERP42();
        ///
        //! @brief Initialize buffers, node publishers, parameters.
        //!
        //! @param[in] nh private namespace node handle for this node
        ///
        void Initialize(ros::NodeHandle &nh);
        ///
        //! @brief Callback for command message
        //!
        //! @param[in] msg erp42 command message
        ///
        void CommandCallback(const rr_common::ERP42CommandRawConstPtr msg);
        ///
        //! @brief Read from the serial device and publish a ERP42FeedbackRaw message
        //!
        //! @return true read succeed and message is published
        //! @return false read failed
        ///
        bool ReadUpdate();
        ///
        //! @brief Make Packet from the latest ERP42CommandRaw message.
        //! Write packet to the serial device. alive is may not be same as the message.
        //!
        //! @return true packet sent properly.
        //! @return false write failed.
        ///
        bool WriteUpdate();
        ///
        //! @brief do read update and write update.
        ///
        void SpinOnce();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif //RR_DEVICES_RR_ERP42_ERP42_H_
