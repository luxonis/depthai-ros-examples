
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <cstdio>
// #include "utility.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
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
    stereo->initialConfig.setConfidenceThreshold(230);
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

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rgb_stereo_node");
    
    std::string deviceName;
    std::string camera_param_uri = "package://depthai_examples/params/camera";
    bool lrcheck, extended, subpixel;

    node->declare_parameter("camera_name", "oak");
    node->declare_parameter("camera_param_uri", camera_param_uri);
    node->declare_parameter("lrcheck", true);
    node->declare_parameter("extended", false);
    node->declare_parameter("subpixel", true);

    node->get_parameter("camera_name", deviceName);
    node->get_parameter("camera_param_uri", camera_param_uri);
    node->get_parameter("lrcheck",  lrcheck);
    node->get_parameter("extended",  extended);
    node->get_parameter("subpixel",  subpixel);

    dai::Pipeline pipeline = createPipeline(lrcheck, extended, subpixel);
    dai::Device device(pipeline);

    auto stereoQueue = device.getOutputQueue("depth", 30, false);
    auto previewQueue = device.getOutputQueue("preview", 30, true);

    bool latched_cam_info = true;
    std::string stereo_uri = camera_param_uri + "/" + "right.yaml";
    std::string color_uri = camera_param_uri + "/" + "color.yaml";


    dai::rosBridge::ImageConverter depthConverter(deviceName + "_right_camera_optical_frame", true);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(stereoQueue,
                                                                                     node, 
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
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(previewQueue,
                                                                                    node, 
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

    rclcpp::spin(node);

    return 0;
}

