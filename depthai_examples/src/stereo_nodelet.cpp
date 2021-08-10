#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include <depthai_examples/stereo_pipeline.hpp>
#include <functional>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

namespace depthai_examples{


 class StereoNodelet : public nodelet::Nodelet
{

    std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>> leftPublish, rightPublish, depthPublish;
    std::unique_ptr<dai::rosBridge::ImageConverter> leftConverter, rightConverter;
    std::unique_ptr<dai::Device> _dev;

    public:
        virtual void onInit() override {

            auto& pnh = getPrivateNodeHandle();
            
            std::string deviceName, mode;
            std::string cameraParamUri;
            int badParams = 0;
            bool lrcheck, extended, subpixel, enableDepth;

            badParams += !pnh.getParam("camera_name", deviceName);
            badParams += !pnh.getParam("camera_param_uri", cameraParamUri);
            badParams += !pnh.getParam("mode", mode);
            badParams += !pnh.getParam("lrcheck",  lrcheck);
            badParams += !pnh.getParam("extended",  extended);
            badParams += !pnh.getParam("subpixel",  subpixel);
            
            if (badParams > 0)
            {   
                std::cout << " Bad parameters -> " << badParams << std::endl;
                throw std::runtime_error("Couldn't find %d of the parameters");
            }

            if(mode == "depth"){
                enableDepth = true;
            }
            else{
                enableDepth = false;
            }

            dai::Pipeline pipeline = createPipeline(enableDepth, lrcheck, extended, subpixel);
            _dev = std::make_unique<dai::Device>(pipeline);

            auto leftQueue = _dev->getOutputQueue("left", 30, false);
            auto rightQueue = _dev->getOutputQueue("right", 30, false);

            std::shared_ptr<dai::DataOutputQueue> stereoQueue;
            if (enableDepth) {
                stereoQueue = _dev->getOutputQueue("depth", 30, false);
            }else{
                stereoQueue = _dev->getOutputQueue("disparity", 30, false);
            }
            auto calibrationHandler = _dev->readCalibration();

            // this part would be removed once we have calibration-api
            /*             
            std::string left_uri = camera_param_uri +"/" + "left.yaml";

            std::string right_uri = camera_param_uri + "/" + "right.yaml";

            std::string stereo_uri = camera_param_uri + "/" + "right.yaml"; 
            */

            leftConverter = std::make_unique<dai::rosBridge::ImageConverter>(deviceName + "_left_camera_optical_frame", true);
            auto leftCameraInfo = leftConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, 1280, 720); 

            leftPublish  = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>
                                                                                            (leftQueue,
                                                                                             pnh, 
                                                                                             std::string("left/image"),
                                                                                             std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                             leftConverter.get(),
                                                                                             std::placeholders::_1, 
                                                                                             std::placeholders::_2) , 
                                                                                             30,
                                                                                             leftCameraInfo,
                                                                                             "left");

            // bridgePublish.startPublisherThread();
            leftPublish->addPubisherCallback();

            rightConverter = std::make_unique<dai::rosBridge::ImageConverter >(deviceName + "_right_camera_optical_frame", true);
            auto rightCameraInfo = rightConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, 1280, 720); 

            rightPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>
                                                                                            (rightQueue,
                                                                                             pnh, 
                                                                                             std::string("right/image"),
                                                                                             std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                             rightConverter.get(), 
                                                                                             std::placeholders::_1, 
                                                                                             std::placeholders::_2) , 
                                                                                             30,
                                                                                             rightCameraInfo,
                                                                                             "right");

            rightPublish->addPubisherCallback();

            // dai::rosBridge::ImageConverter depthConverter(deviceName + "_right_camera_optical_frame");
            depthPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>
                                                                            (stereoQueue,
                                                                             pnh, 
                                                                             std::string("stereo/depth"),
                                                                             std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                             rightConverter.get(), // since the converter has the same frame name
                                                                                             // and image type is also same we can reuse it
                                                                             std::placeholders::_1, 
                                                                             std::placeholders::_2) , 
                                                                             30,
                                                                             rightCameraInfo,
                                                                             "stereo");

            depthPublish->addPubisherCallback();

            // We can add the rectified frames also similar to these publishers. 
            // Left them out so that users can play with it by adding and removing
        }


    dai::Pipeline createPipeline(bool withDepth, bool lrcheck, bool extended, bool subpixel){
        dai::Pipeline pipeline;

        auto monoLeft    = pipeline.create<dai::node::MonoCamera>();
        auto monoRight   = pipeline.create<dai::node::MonoCamera>();
        auto xoutLeft    = pipeline.create<dai::node::XLinkOut>();
        auto xoutRight   = pipeline.create<dai::node::XLinkOut>();
        auto stereo      = pipeline.create<dai::node::StereoDepth>();
        auto xoutDepth   = pipeline.create<dai::node::XLinkOut>();

        // XLinkOut
        xoutLeft->setStreamName("left");
        xoutRight->setStreamName("right");

        if (withDepth) {
            xoutDepth->setStreamName("depth");
        }
        else {
            xoutDepth->setStreamName("disparity");
        }

        // MonoCamera
        monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

        // int maxDisp = 96;
        // if (extended) maxDisp *= 2;
        // if (subpixel) maxDisp *= 32; // 5 bits fractional disparity

        // StereoDepth
        // stereo->initialConfig.setConfidenceThreshold(200);
        stereo->initialConfig.setLeftRightCheckThreshold(1);
        stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout

        stereo->setLeftRightCheck(lrcheck);
        stereo->setExtendedDisparity(extended);
        stereo->setSubpixel(subpixel);

        // Link plugins CAM -> STEREO -> XLINK
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);

        stereo->syncedLeft.link(xoutLeft->input);
        stereo->syncedRight.link(xoutRight->input);

        if(withDepth){
            stereo->depth.link(xoutDepth->input);
        }
        else{
            stereo->disparity.link(xoutDepth->input);
        }

        return pipeline;
    }
};

PLUGINLIB_EXPORT_CLASS(depthai_examples::StereoNodelet, nodelet::Nodelet)
}   // namespace depthai_examples
