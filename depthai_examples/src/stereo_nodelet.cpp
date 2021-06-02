#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include <depthai_examples/stereo_pipeline.hpp>
#include <functional>

// #include <depthai_examples/daiUtility.hpp>
// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

namespace depthai_examples{


 class StereoNodelet : public nodelet::Nodelet
{

    std::unique_ptr<StereoExampe> stereo_pipeline;
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues;
    std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>> leftPublish, rightPublish, depthPublish;
    std::unique_ptr<dai::rosBridge::ImageConverter> leftConverter, rightConverter;

    public:
        virtual void onInit() override {

            auto& pnh = getPrivateNodeHandle();
            
            std::string deviceName;
            std::string camera_param_uri;
            int bad_params = 0;

            bad_params += !pnh.getParam("camera_name", deviceName);
            bad_params += !pnh.getParam("camera_param_uri", camera_param_uri);

            if (bad_params > 0)
            {
                throw std::runtime_error("Couldn't find one of the parameters");
            }

            stereo_pipeline = std::make_unique<StereoExampe>();
            stereo_pipeline->initDepthaiDev();
            imageDataQueues = stereo_pipeline->getExposedImageStreams();
            
            std::vector<ros::Publisher> imgPubList;
            std::vector<std::string> frameNames;
            
            // this part would be removed once we have calibration-api
            std::string left_uri = camera_param_uri +"/" + "left.yaml";

            std::string right_uri = camera_param_uri + "/" + "right.yaml";

            std::string stereo_uri = camera_param_uri + "/" + "right.yaml";

            leftConverter = std::make_unique<dai::rosBridge::ImageConverter>(deviceName + "_left_camera_optical_frame", true);
            leftPublish  = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>
                (imageDataQueues[0],
                pnh, 
                std::string("left/image"),
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                leftConverter.get(),
                std::placeholders::_1, 
                std::placeholders::_2) , 
                1,
                left_uri,
                "left");

            // bridgePublish.startPublisherThread();
            leftPublish->addPubisherCallback();

            rightConverter = std::make_unique<dai::rosBridge::ImageConverter >(deviceName + "_right_camera_optical_frame", true);
            rightPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>
                (imageDataQueues[1],
                pnh, 
                std::string("right/image"),
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                rightConverter.get(), 
                std::placeholders::_1, 
                std::placeholders::_2) , 
                1,
                right_uri,
                "right");

            rightPublish->addPubisherCallback();

            // dai::rosBridge::ImageConverter depthConverter(deviceName + "_right_camera_optical_frame");
            depthPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>
                (imageDataQueues[2],
                pnh, 
                std::string("stereo/depth"),
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                rightConverter.get(), // since the converter has the same frame name
                                // and image type is also same we can reuse it
                std::placeholders::_1, 
                std::placeholders::_2) , 
                1,
                stereo_uri,
                "stereo");

            depthPublish->addPubisherCallback();

            // We can add the rectified frames also similar to these publishers. 
            // Left them out so that users can play with it by adding and removing
        }
};

//PLUGINLIB_EXPORT_CLASS(depthai_examples::StereoNodelet, nodelet::Nodelet)


}   // namespace depthai_examples
PLUGINLIB_EXPORT_CLASS(depthai_examples::StereoNodelet, nodelet::Nodelet)



