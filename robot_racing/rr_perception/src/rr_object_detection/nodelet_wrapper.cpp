#include <pluginlib/class_list_macros.h>

#include "rr_object_detection/object_detector.h"
#include <nodelet/nodelet.h>
#include <memory>

namespace rr_perception
{

    class ObjectDetectorNodelet : public nodelet::Nodelet
    {
    public:
        virtual void onInit();
    };
    void ObjectDetectorNodelet::onInit()
    {
        ObjectDetector filter(getPrivateNodeHandle());
        // filter.Run();
    }
}

PLUGINLIB_EXPORT_CLASS(rr_perception::ObjectDetectorNodelet, nodelet::Nodelet)
