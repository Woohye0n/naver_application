#include "rr_vms/drive_mode_checker.h"
#include "rr_common/PIControllerOnOff.h"
namespace rr_vms
{
    struct DriveModeChecker::Impl
    {
        ros::ServiceClient pid_on_off_client;
        uint8_t drive_mode;
        std::vector<std::string> mode_strings;
    };
    DriveModeChecker::DriveModeChecker() : impl_(new Impl) {}
    DriveModeChecker::~DriveModeChecker() {}
    void DriveModeChecker::Init(ros::NodeHandle &nh)
    {
        impl_->pid_on_off_client = nh.serviceClient<rr_common::PIControllerOnOff>("pi_controller_on_off");
        impl_->mode_strings.resize(2);
        impl_->drive_mode = rr_common::ERP42FeedbackRaw::kModeManual;
        impl_->mode_strings[rr_common::ERP42FeedbackRaw::kModeAuto] = "Auto";
        impl_->mode_strings[rr_common::ERP42FeedbackRaw::kModeManual] = "Manual";
    }
    bool DriveModeChecker::IsOk()
    {
        return impl_->drive_mode == rr_common::ERP42FeedbackRaw::kModeAuto;
    }
    void DriveModeChecker::CheckMessage(const rr_common::ERP42FeedbackRaw::Ptr &msg)
    {
        const uint8_t &prev_mode = impl_->drive_mode;
        const uint8_t &new_mode = msg->a_or_m;
        const uint8_t &kModeAuto = rr_common::ERP42FeedbackRaw::kModeAuto;
        const uint8_t &kModeManual = rr_common::ERP42FeedbackRaw::kModeManual;

        rr_common::PIControllerOnOff srv;
        // auto to manual. turn off the pi control
        if (prev_mode == kModeAuto && new_mode == kModeManual)
        {
            srv.request.header.stamp = ros::Time::now();
            srv.request.header.frame_id = "vms";
            srv.request.is_on = srv.request.kOff;
            impl_->pid_on_off_client.call(srv);
        }
        // manual to auto. turn on the pi control
        else if (prev_mode == kModeManual && new_mode == kModeAuto)
        {
            srv.request.header.stamp = ros::Time::now();
            srv.request.header.frame_id = "vms";
            srv.request.is_on = srv.request.kOn;
            impl_->pid_on_off_client.call(srv);
        }

        // update prev mode
        impl_->drive_mode = msg->a_or_m;
    }
    std::string DriveModeChecker::ModeString()
    {
        return impl_->mode_strings[impl_->drive_mode];
    }
} // namespace rr_vms