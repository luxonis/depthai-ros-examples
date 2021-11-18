
#include "rclcpp/rclcpp.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
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
    stereo->initialConfig.setConfidenceThreshold(230);
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(10);
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

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_node");
    
    std::string deviceName, mode;
    std::string cameraParamUri;
    int strobe_mode = 0, badParams = 0;
    bool lrcheck, extended, subpixel, enableDepth;

    /* badParams += !node->get_parameter("camera_name", deviceName);
    badParams += !node->get_parameter("camera_param_uri", cameraParamUri);
    badParams += !node->get_parameter("mode", mode);
    badParams += !node->get_parameter("lrcheck",  lrcheck);
    badParams += !node->get_parameter("extended",  extended);
    badParams += !node->get_parameter("subpixel",  subpixel);

    if (badParams > 0)
    {
        std::cout << " Bad parameters -> " << badParams << std::endl;
        throw std::runtime_error("Couldn't find one of the parameters");
    } */

    deviceName = "OAK-d";
    mode = "depth";
    lrcheck = true;
    extended = false;
    subpixel = true;

    if(mode == "depth"){
        enableDepth = true;
    }
    else{
        enableDepth = false;
    }

    dai::Pipeline pipeline = createPipeline(enableDepth, lrcheck, extended, subpixel);

    dai::Device device(pipeline);
    device.irWriteReg(0x2, 0x1)
    device.irWriteReg(0x3, 0x0)
    device.irWriteReg(0x4, 0x0)
    device.irWriteReg(0x5, 0x0)
    device.irWriteReg(0x6, 0x0)
    device.irWriteReg(0x7, 0x9)
    device.irWriteReg(0x8, 0x1a)
    device.irWriteReg(0x9, 0x8)
    device.irWriteReg(0x1, 0x24)
    auto leftQueue = device.getOutputQueue("left", 30, false);
    auto rightQueue = device.getOutputQueue("right", 30, false);
    std::shared_ptr<dai::DataOutputQueue> stereoQueue;
    if (enableDepth) {
        stereoQueue = device.getOutputQueue("depth", 30, false);
    }else{
        stereoQueue = device.getOutputQueue("disparity", 30, false);
    }

    auto calibrationHandler = device.readCalibration();
    std::cout << "Press +/- to increase and decreasesdadasddd brightness" << std::endl;

    // this part would be removed once we have calibration-api
    /*     
     std::string leftUri = cameraParamUri +"/" + "left.yaml";

     std::string rightUri = cameraParamUri + "/" + "right.yaml";

     std::string stereoUri = cameraParamUri + "/" + "right.yaml";
    */        
    dai::rosBridge::ImageConverter converter(deviceName + "_left_camera_optical_frame", true);
    auto leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, 1280, 720); 
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(leftQueue,
                                                                                    node, 
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

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(rightQueue,
                                                                                     node, 
                                                                                     std::string("right/image"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rightconverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     rightCameraInfo,
                                                                                     "right");

    rightPublish.addPubisherCallback();
    
    std::cout << "Enabling Strobe and Projector" << std::endl;
    int enable =  strobe_mode << 6 | 1 << 5 | 1 << 1 | 1 << 2; 
    device.irWriteReg(0x01, enable);

     if(mode == "depth"){

        dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(stereoQueue,
                                                                                     node, 
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
        int brightness = 54;
        int oldBrightness = brightness;
        device.irWriteReg(0x04, brightness);
        char key;
        while (rclcpp::ok()){
            rclcpp::spin_some(node);
            key = getchar();

            if(key == '+'){
                brightness += 3;
            }
            else if(key == '-'){
                brightness -= 3;
            }
            else if(key == EOF){
                continue;
            }
            else{
                std::cout << "Press +/- to increase and decrease brightness" << std::endl;
            }

            if (brightness > 127){
                brightness = 127;
                std::cout << "Max available brightness is 127. cannot increment beyond that" << std::endl;
            }
            else if (brightness < 0){
                brightness = 0;
                std::cout << "Min available brightness is 0. cannot decrement below 0" << std::endl;
            }

            if (oldBrightness != brightness){
                device.irWriteReg(0x04, brightness);
                oldBrightness = brightness;
                std::cout << "Updated Brightness " << brightness << std::endl;
            }
        }
    }
    else{
        dai::rosBridge::DisparityConverter dispConverter(deviceName + "_right_camera_optical_frame", 880, 7.5, 20, 2000);
        dai::rosBridge::BridgePublisher<stereo_msgs::msg::DisparityImage, dai::ImgFrame> dispPublish(stereoQueue,
                                                                                     node, 
                                                                                     std::string("stereo/disparity"),
                                                                                     std::bind(&dai::rosBridge::DisparityConverter::toRosMsg, 
                                                                                     &dispConverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     rightCameraInfo,
                                                                                     "stereo");
        dispPublish.addPubisherCallback();
    }
    // rclcpp::spin(node);
    // int brightness = 54;
    // dev.irWriteReg(0x04, brightness);
    // std::cout << "Press +/- to increase and decrease brightness" << std::endl;
    

    /* char key;
    while (rclcpp::ok()){
        rclcpp::spin_some(node);
        
        // We can add the rectified frames also similar to these publishers. 
        // Left them out so that users can play with it by adding and removing
        key = getchar();

        if(key == '+'){
            brightness += 3;
        }
        else if(key == '-'){
            brightness -= 3;
        }
        else if(key == EOF){
            continue;
        }
        else{
            std::cout << "Press +/- to increase and decrease brightness" << std::endl;
        }

        if (brightness > 127){
            brightness = 127;
            std::cout << "Max available brightness is 127. cannot increment beyond that" << std::endl;
        }
        else if (brightness < 0){
            brightness = 0;
            std::cout << "Min available brightness is 0. cannot decrement below 0" << std::endl;
        }

    } */
    
    return 0;
}
