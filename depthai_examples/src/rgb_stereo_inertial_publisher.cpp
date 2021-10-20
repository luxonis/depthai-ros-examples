
#include "ros/ros.h"

#include <iostream>
#include <cstdio>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"
#include <camera_info_manager/camera_info_manager.h>
#include <functional>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImuConverter.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/DisparityConverter.hpp>


dai::Pipeline createPipeline(bool withDepth, bool lrcheck, bool extended, bool subpixel, bool rectify, bool depth_aligned){
    dai::Pipeline pipeline;

    auto camRgb               = pipeline.create<dai::node::ColorCamera>();
    auto xoutRgb              = pipeline.create<dai::node::XLinkOut>();
    auto monoLeft             = pipeline.create<dai::node::MonoCamera>();
    auto monoRight            = pipeline.create<dai::node::MonoCamera>();
    auto xoutLeft             = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight            = pipeline.create<dai::node::XLinkOut>();
    auto xoutRectifiedLeft    = pipeline.create<dai::node::XLinkOut>();
    auto xoutRectifiedRight   = pipeline.create<dai::node::XLinkOut>();
    auto stereo               = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth            = pipeline.create<dai::node::XLinkOut>();
    auto imu                  = pipeline.create<dai::node::IMU>();
    auto xoutImu              = pipeline.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutRgb->setStreamName("rgb");
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    xoutRectifiedLeft->setStreamName("rectifiedleft");
    xoutRectifiedRight->setStreamName("rectifiedright");

    if (withDepth) {
        xoutDepth->setStreamName("depth");
    }
    else {
        xoutDepth->setStreamName("disparity");
    }

    xoutImu->setStreamName("imu");

    //RGB
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    if(withDepth && depth_aligned){
        // the ColorCamera is downscaled from 1080p to 720p.
        // Otherwise, the aligned depth is automatically upscaled to 1080p
        camRgb->setIspScale(2, 3);
        // For now, RGB needs fixed focus to properly align with depth.
        // This value was used during calibration
        camRgb->initialControl.setManualFocus(135);
    }

    // MonoCamera
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(200);
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(1);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);
    if(withDepth && depth_aligned) stereo->setDepthAlign(dai::CameraBoardSocket::RGB);

    //Imu
    imu->enableIMUSensor({dai::IMUSensor::ROTATION_VECTOR, dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 400);
    imu->setMaxBatchReports(1); // Get one message only for now.

    // Link plugins CAM -> STEREO -> XLINK
    camRgb->isp.link(xoutRgb->input);

    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    if(rectify){
        stereo->rectifiedLeft.link(xoutLeft->input);
        stereo->rectifiedRight.link(xoutRight->input);     
    }else{
        stereo->syncedLeft.link(xoutLeft->input);
        stereo->syncedRight.link(xoutRight->input);
    }

    if(withDepth){
        stereo->depth.link(xoutDepth->input);
    }
    else{
        stereo->disparity.link(xoutDepth->input);
    }

    imu->out.link(xoutImu->input);

    return pipeline;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "rgb_stereo_inertial_node");
    ros::NodeHandle pnh("~");
    
    std::string deviceName, mode;
    int badParams = 0;
    bool lrcheck, extended, subpixel, enableDepth, rectify, depth_aligned;

    badParams += !pnh.getParam("camera_name", deviceName);
    badParams += !pnh.getParam("mode", mode);
    badParams += !pnh.getParam("lrcheck",  lrcheck);
    badParams += !pnh.getParam("extended",  extended);
    badParams += !pnh.getParam("subpixel",  subpixel);
    badParams += !pnh.getParam("rectify",  rectify);
    badParams += !pnh.getParam("depth_aligned",  depth_aligned);

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

    dai::Pipeline pipeline = createPipeline(enableDepth, lrcheck, extended, subpixel, rectify, depth_aligned);

    dai::Device device(pipeline);

    auto imgQueue = device.getOutputQueue("rgb", 30, false);
    auto leftQueue = device.getOutputQueue("left", 30, false);
    auto rightQueue = device.getOutputQueue("right", 30, false);
    std::shared_ptr<dai::DataOutputQueue> stereoQueue;
    if (enableDepth) {
        stereoQueue = device.getOutputQueue("depth", 30, false);
    }else{
        stereoQueue = device.getOutputQueue("disparity", 30, false);
    }
    auto imuQueue = device.getOutputQueue("imu",30,false);

    auto calibrationHandler = device.readCalibration();

    dai::rosBridge::ImageConverter rgbConverter(deviceName + "_rgb_camera_optical_frame", false);
    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, 1920, 1080); 
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(imgQueue,
                                                                                  pnh, 
                                                                                  std::string("color/image"),
                                                                                  std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                  &rgbConverter, // since the converter has the same frame name
                                                                                                  // and image type is also same we can reuse it
                                                                                  std::placeholders::_1, 
                                                                                  std::placeholders::_2) , 
                                                                                  30,
                                                                                  rgbCameraInfo,
                                                                                  "color");
    rgbPublish.addPubisherCallback();
    
    const std::string leftPubName = rectify?std::string("left/image_rect"):std::string("left/image_raw");
        
    dai::rosBridge::ImageConverter converter(deviceName + "_left_camera_optical_frame", true);
    auto leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, 1280, 720); 
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftPublish(leftQueue,
                                                                                    pnh, 
                                                                                    leftPubName,
                                                                                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                    &converter, 
                                                                                    std::placeholders::_1, 
                                                                                    std::placeholders::_2) , 
                                                                                    30,
                                                                                    leftCameraInfo,
                                                                                    "left");

    leftPublish.addPubisherCallback();

    const std::string rightPubName = rectify?std::string("right/image_rect"):std::string("right/image_raw");

    dai::rosBridge::ImageConverter rightconverter(deviceName + "_right_camera_optical_frame", true);
    auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, 1280, 720); 

    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rightPublish(rightQueue,
                                                                                     pnh, 
                                                                                     rightPubName,
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rightconverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     rightCameraInfo,
                                                                                     "right");

    rightPublish.addPubisherCallback();


    // We can add the rectified frames also similar to these publishers. 
    // Left them out so that users can play with it by adding and removing

    dai::rosBridge::ImuConverter imuConverter(deviceName +"_imu_frame");

    dai::rosBridge::BridgePublisher<sensor_msgs::Imu, dai::IMUData> ImuPublish(imuQueue,
                                                                                     pnh, 
                                                                                     std::string("imu"),
                                                                                     std::bind(&dai::rosBridge::ImuConverter::toRosMsg, 
                                                                                     &imuConverter, 
                                                                                     std::placeholders::_1,
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     "",
                                                                                     "imu");

    ImuPublish.startPublisherThread();
    
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
    
    return 0;
}