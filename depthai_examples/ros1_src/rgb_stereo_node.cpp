
#include "ros/ros.h"

#include <iostream>
#include <cstdio>
// #include "utility.hpp"
#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include <depthai_examples/rgb_stereo_pipeline.hpp>
#include <functional>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

dai::Pipeline createPipeline(bool lrcheck, bool extended, bool subpixel){

    dai::Pipeline pipeline;
    auto monoLeft    = pipeline.create<dai::node::MonoCamera>();
    auto monoRight   = pipeline.create<dai::node::MonoCamera>();
    
    auto stereo      = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth   = pipeline.create<dai::node::XLinkOut>();

    xoutDepth->setStreamName("depth");
  
    // MonoCamera
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    // StereoDepth
    stereo->setConfidenceThreshold(200);
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout

    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    // // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    stereo->depth.link(xoutDepth->input);

    // Color camers steream setup -------->
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("preview");
    
    colorCam->setPreviewSize(1920, 1080);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(true);

    // Link plugins CAM -> XLINK
    colorCam->preview.link(xlinkOut->input);

    return pipeline;
}


int main(int argc, char** argv){

    ros::init(argc, argv, "rgb_stereo_node");
    ros::NodeHandle pnh("~");
    
    std::string deviceName;
    std::string camera_param_uri;
    int bad_params = 0;
    bool lrcheck, extended, subpixel;

    bad_params += !pnh.getParam("camera_name", deviceName);
    bad_params += !pnh.getParam("camera_param_uri", camera_param_uri);
    bad_params += !pnh.getParam("lrcheck",  lrcheck);
    bad_params += !pnh.getParam("extended",  extended);
    bad_params += !pnh.getParam("subpixel",  subpixel);

    if (bad_params > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }
    dai::Pipeline pipeline = createPipeline(lrcheck, extended, subpixel);
    dai::Device device(pipeline);

    auto stereoQueue = device.getOutputQueue("depth", 30, false);
    auto previewQueue = device.getOutputQueue("preview", 30, true);

    bool latched_cam_info = true;
    std::string stereo_uri = camera_param_uri + "/" + "right.yaml";
    std::string color_uri = camera_param_uri + "/" + "color.yaml";


    dai::rosBridge::ImageConverter depthConverter(deviceName + "_right_camera_optical_frame", true);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(stereoQueue,
                                                                                     pnh, 
                                                                                     std::string("stereo/depth"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &depthConverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     stereo_uri,
                                                                                     "stereo");


    dai::rosBridge::ImageConverter rgbConverter(deviceName + "_rgb_camera_optical_frame", true);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(previewQueue,
                                                                                    pnh, 
                                                                                    std::string("color/image"),
                                                                                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                    &rgbConverter, // since the converter has the same frame name
                                                                                                    // and image type is also same we can reuse it
                                                                                    std::placeholders::_1, 
                                                                                    std::placeholders::_2) , 
                                                                                    30,
                                                                                    color_uri,
                                                                                    "color");

    depthPublish.addPubisherCallback(); // addPubisherCallback works only when the dataqueue is non blocking.
    rgbPublish.addPubisherCallback();

    // We can add the rectified frames also similar to these publishers. 
    // Left them out so that users can play with it by adding and removing

    ros::spin();

    return 0;
}

