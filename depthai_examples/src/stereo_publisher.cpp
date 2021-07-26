
#include "ros/ros.h"

#include <iostream>
#include <cstdio>
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"
#include <camera_info_manager/camera_info_manager.h>
#include <depthai_examples/stereo_pipeline.hpp>
#include <functional>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/DisparityConverter.hpp>


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
    stereo->initialConfig.setConfidenceThreshold(200);
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

int main(int argc, char** argv){

    ros::init(argc, argv, "stereo_node");
    ros::NodeHandle pnh("~");
    
    std::string deviceName, mode;
    std::string cameraParamUri;
    int badParams = 0;
    bool lrcheck, extended, subpixel, enableDepth;

    badParams += !pnh.getParam("camera_name", deviceName);
    badParams += !pnh.getParam("cameraParamUri", cameraParamUri);
    badParams += !pnh.getParam("mode", mode);
    badParams += !pnh.getParam("lrcheck",  lrcheck);
    badParams += !pnh.getParam("extended",  extended);
    badParams += !pnh.getParam("subpixel",  subpixel);
    

    if (badParams > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }

    StereoExampe stereo_pipeline;
    if(mode == "depth"){
        enableDepth = true;
    }
    else{
        enableDepth = false;
    }

    dai::Pipeline pipeline = createPipeline(enableDepth, lrcheck, extended, subpixel);

    dai::Device device(pipeline);


    auto leftQueue = device.getOutputQueue("left", 30, false);
    auto rightQueue = device.getOutputQueue("right", 30, false);
    std::shared_ptr<dai::DataOutputQueue> stereoQueue;
    if (enableDepth) {
        stereoQueue = device.getOutputQueue("depth", 30, false);
    }else{
        stereoQueue = device.getOutputQueue("disparity", 30, false);
    }
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues = stereo_pipeline.getExposedImageStreams();

    auto calibrationHandler = device.readCalibration();

    // this part would be removed once we have calibration-api
    std::string leftUri = cameraParamUri +"/" + "left.yaml";
  
     std::string rightUri = cameraParamUri + "/" + "right.yaml";
    
     std::string stereoUri = cameraParamUri + "/" + "right.yaml";
    
    
    dai::rosBridge::ImageConverter converter(deviceName + "_left_camera_optical_frame", true);
    auto leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, 1280, 720); 
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftPublish(leftQueue,
                                                                                    pnh, 
                                                                                    std::string("left/image"),
                                                                                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                    &converter, 
                                                                                    std::placeholders::_1, 
                                                                                    std::placeholders::_2) , 
                                                                                    30,
                                                                                    leftCameraInfo,
                                                                                    "left");

    leftPublish.addPubisherCallback();

    dai::rosBridge::ImageConverter rightconverter(deviceName + "_right_camera_optical_frame", true);
    auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, 1280, 720); 

    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rightPublish(rightQueue,
                                                                                     pnh, 
                                                                                     std::string("right/image"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rightconverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     rightCameraInfo,
                                                                                     "right");

    rightPublish.addPubisherCallback();

     if(mode == "depth"){
         std::cout << "In depth";
        dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(stereoQueue,
                                                                                     pnh, 
                                                                                     std::string("stereo/depth"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rightconverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     rightCameraInfo,
                                                                                     "stereo");
        depthPublish.addPubisherCallback();
        ros::spin();
    }
    else{
        dai::rosBridge::DisparityConverter dispConverter(deviceName + "_right_camera_optical_frame", 880, 7.5, 20, 2000);
        dai::rosBridge::BridgePublisher<stereo_msgs::DisparityImage, dai::ImgFrame> dispPublish(stereoQueue,
                                                                                     pnh, 
                                                                                     std::string("stereo/disparity"),
                                                                                     std::bind(&dai::rosBridge::DisparityConverter::toRosMsg, 
                                                                                     &dispConverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     rightCameraInfo,
                                                                                     "stereo");
        dispPublish.addPubisherCallback();
        ros::spin();
    }

    // We can add the rectified frames also similar to these publishers. 
    // Left them out so that users can play with it by adding and removing

    
    return 0;
}
