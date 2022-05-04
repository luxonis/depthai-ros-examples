#include <camera_info_manager/camera_info_manager.h>

// #include <memory>
#include <camera_info_manager/camera_info_manager.h>

#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "stereo_msgs/DisparityImage.h"

// Inludes common necessary includes for development using depthai library
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/DisparityConverter.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/ImuConverter.hpp>

// #include "depthai_bridge/CameraControl.hpp"
#include "depthai_bridge/DepthPostProcessing.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "depthai/depthai.hpp"

using namespace dai::ros;

std::tuple<dai::Pipeline, int, int> createPipeline(bool enableDepth,
                                                   bool lrcheck,
                                                   bool extended,
                                                   bool subpixel,
                                                   bool rectify,
                                                   bool depthAligned,
                                                   int stereoFps,
                                                   int confidence,
                                                   int LRchecktresh,
                                                   std::string resolution,
                                                   DepthPostProcessing postProcessing) {
    dai::Pipeline pipeline;

    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto imu = pipeline.create<dai::node::IMU>();
    auto xoutImu = pipeline.create<dai::node::XLinkOut>();

    if(enableDepth) {
        xoutDepth->setStreamName("depth");
    } else {
        xoutDepth->setStreamName("disparity");
    }

    xoutImu->setStreamName("imu");

    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    int width, height;
    if(resolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
        width = 1280;
        height = 720;
    } else if(resolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
        width = 640;
        height = 400;
    } else if(resolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
        width = 1280;
        height = 800;
    } else if(resolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
        width = 640;
        height = 480;
    } else {
        ROS_ERROR("Invalid parameter. -> monoResolution: %s", resolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    // MonoCamera
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoLeft->setFps(stereoFps);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoRight->setFps(stereoFps);

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(confidence);        // Known to be best
    stereo->setRectifyEdgeFillColor(0);                              // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);  // Known to be best
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);
    auto config = postProcessing.getFilters(stereo->initialConfig.get());
    stereo->initialConfig.set(config);
    if(depthAligned) stereo->setDepthAlign(dai::CameraBoardSocket::RGB);

    // Imu
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(5);
    imu->setMaxBatchReports(20);

    if(depthAligned) {
        // RGB image
        auto camRgb = pipeline.create<dai::node::ColorCamera>();
        auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
        xoutRgb->setStreamName("rgb");
        camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
        camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        // the ColorCamera is downscaled from 1080p to 720p.
        // Otherwise, the aligned depth is automatically upscaled to 1080p
        if(height < 720) {
            camRgb->setIspScale(1, 3);
        } else {
            camRgb->setIspScale(2, 3);
        }
        camRgb->isp.link(xoutRgb->input);
        // auto rgbControlIn = pipeline.create<dai::node::XLinkIn>();
        // rgbControlIn->setStreamName("control_rgb");
        // rgbControlIn->out.link(camRgb->inputControl);
        // auto rgbConfigIn = pipeline.create<dai::node::XLinkIn>();
        // rgbConfigIn->setStreamName("config_rgb");
        // rgbConfigIn->out.link(camRgb->inputConfig);
    } else {
        // Stereo imges
        auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
        auto xoutRight = pipeline.create<dai::node::XLinkOut>();
        // XLinkOut
        xoutLeft->setStreamName("left");
        xoutRight->setStreamName("right");
        if(rectify) {
            stereo->rectifiedLeft.link(xoutLeft->input);
            stereo->rectifiedRight.link(xoutRight->input);
        } else {
            stereo->syncedLeft.link(xoutLeft->input);
            stereo->syncedRight.link(xoutRight->input);
        }
    }

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    auto stereoControlIn = pipeline.create<dai::node::XLinkIn>();
    stereoControlIn->setStreamName("stereoDepthConfig");
    stereoControlIn->out.link(stereo->inputConfig);
    // stereoControlIn->out.link(monoRight->inputControl);
    // auto stereoConfigIn = pipeline.create<dai::node::XLinkIn>();
    // stereoConfigIn->setStreamName("config_stereo");
    // stereoConfigIn->out.link(stereo->inputConfig);
    // stereoConfigIn->out.link(monoRight->inputConfig);

    if(enableDepth) {
        stereo->depth.link(xoutDepth->input);
    } else {
        stereo->disparity.link(xoutDepth->input);
    }

    imu->out.link(xoutImu->input);
    return std::make_tuple(pipeline, width, height);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "stereo_inertial_node");
    ros::NodeHandle pnh("~");

    std::string tfPrefix = "oak", mode = "depth";
    std::string monoResolution = "720p";
    int stereoFps = 15, confidence = 200, LRchecktresh = 5, imuModeParam = 1;
    bool lrcheck = true, extended = false, subpixel = true, rectify = true, depthAligned = true;
    float angularVelCovariance = 0.02, linearAccelCovariance = 0.0;

    dai::ros::getParamWithWarning(pnh, "tf_prefix", tfPrefix);
    dai::ros::getParamWithWarning(pnh, "mode", mode);
    dai::ros::getParamWithWarning(pnh, "monoResolution", monoResolution);
    dai::ros::getParamWithWarning(pnh, "imuMode", imuModeParam);
    dai::ros::getParamWithWarning(pnh, "angularVelCovariance", angularVelCovariance);
    dai::ros::getParamWithWarning(pnh, "linearAccelCovariance", linearAccelCovariance);
    dai::ros::getParamWithWarning(pnh, "stereoFps", stereoFps);
    dai::ros::getParamWithWarning(pnh, "confidence", confidence);
    dai::ros::getParamWithWarning(pnh, "LRchecktresh", LRchecktresh);
    dai::ros::getParamWithWarning(pnh, "lrcheck", lrcheck);
    dai::ros::getParamWithWarning(pnh, "extended", extended);
    dai::ros::getParamWithWarning(pnh, "subpixel", subpixel);
    dai::ros::getParamWithWarning(pnh, "rectify", rectify);
    dai::ros::getParamWithWarning(pnh, "depthAligned", depthAligned);

        std:: cout << "depthAligned -" << depthAligned << std::endl;

    bool enableDepth;
    if(mode == "depth") {
        enableDepth = true;
    } else {
        enableDepth = false;
    }

    dai::ros::ImuSyncMethod imuMode = static_cast<dai::ros::ImuSyncMethod>(imuModeParam);
    dai::ros::DepthPostProcessing postProcessing(pnh, "stereo");

    dai::Pipeline pipeline;
    int monoWidth, monoHeight;
    std::tie(pipeline, monoWidth, monoHeight) =
        createPipeline(enableDepth, lrcheck, extended, subpixel, rectify, depthAligned, stereoFps, confidence, LRchecktresh, monoResolution, postProcessing);

    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>(pipeline);
    postProcessing.setDevice(device);
    /* 
    dai::ros::CameraControl colorCameraControl(pnh, device, "rgb"), stereoCameraControl(pnh, device, "stereo");
    colorCameraControl.setExposure();
    colorCameraControl.setFocus();
    stereoCameraControl.setExposure();
    stereoCameraControl.setFocus();
    */
    std::shared_ptr<dai::DataOutputQueue> stereoQueue;
    if(enableDepth) {
        stereoQueue = device->getOutputQueue("depth", 30, false);
    } else {
        stereoQueue = device->getOutputQueue("disparity", 30, false);
    }
    auto imuQueue = device->getOutputQueue("imu", 30, false);

    auto calibrationHandler = device->readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if(monoHeight > 480 && boardName == "OAK-D-LITE") {
        monoWidth = 640;
        monoHeight = 480;
    }

    std:: cout << depthAligned << std::endl;
    dai::rosBridge::ImageConverter converter(tfPrefix + "_left_camera_optical_frame", true);
    dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
    auto leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, monoWidth, monoHeight);
    auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, monoWidth, monoHeight);
    const std::string leftPubName = rectify ? std::string("left/image_rect") : std::string("left/image_raw");
    const std::string rightPubName = rectify ? std::string("right/image_rect") : std::string("right/image_raw");

    dai::rosBridge::ImuConverter imuConverter(tfPrefix + "_imu_frame", imuMode, linearAccelCovariance, angularVelCovariance);

    dai::rosBridge::BridgePublisher<sensor_msgs::Imu, dai::IMUData> ImuPublish(
        imuQueue,
        pnh,
        std::string("imu"),
        std::bind(&dai::rosBridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "imu");

    ImuPublish.addPublisherCallback();
    int colorWidth = 1280, colorHeight = 720;
    if(monoHeight < 720) {
        colorWidth = 640;
        colorHeight = 360;
    }
    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, colorWidth, colorHeight);

    if(enableDepth) {
        auto depthCameraInfo =
            depthAligned ? rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, colorWidth, colorHeight) : rightCameraInfo;
        auto depthconverter = depthAligned ? rgbConverter : rightconverter;
        dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(
            stereoQueue,
            pnh,
            std::string("stereo/depth"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                      &depthconverter,  // since the converter has the same frame name
                                        // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
            30,
            depthCameraInfo,
            "stereo");
        depthPublish.addPublisherCallback();

        if(depthAligned) {
            auto imgQueue = device->getOutputQueue("rgb", 30, false);
            dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(
                imgQueue,
                pnh,
                std::string("color/image"),
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
                30,
                rgbCameraInfo,
                "color");
            rgbPublish.addPublisherCallback();
            ros::ServiceServer postProcessingService = pnh.advertiseService("setPostProcessing", &DepthPostProcessing::setDepthPostProcessingRequest, &postProcessing);
            ros::spin();
        } else {
            auto leftQueue = device->getOutputQueue("left", 30, false);
            auto rightQueue = device->getOutputQueue("right", 30, false);
            dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftPublish(
                leftQueue,
                pnh,
                leftPubName,
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &converter, std::placeholders::_1, std::placeholders::_2),
                30,
                leftCameraInfo,
                "left");
            dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rightPublish(
                rightQueue,
                pnh,
                rightPubName,
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rightconverter, std::placeholders::_1, std::placeholders::_2),
                30,
                rightCameraInfo,
                "right");
            rightPublish.addPublisherCallback();
            leftPublish.addPublisherCallback();
            ros::ServiceServer postProcessingService = pnh.advertiseService("setPostProcessing", &DepthPostProcessing::setDepthPostProcessingRequest, &postProcessing);
            ros::spin();
        }
    } else {
        std::string tfSuffix = depthAligned ? "_rgb_camera_optical_frame" : "_right_camera_optical_frame";
        dai::rosBridge::DisparityConverter dispConverter(tfPrefix + tfSuffix, 880, 7.5, 20, 2000);  // TODO(sachin): undo hardcoding of baseline
        auto disparityCameraInfo =
            depthAligned ? rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, 1280, 720) : rightCameraInfo;
        auto depthconverter = depthAligned ? rgbConverter : rightconverter;
        dai::rosBridge::BridgePublisher<stereo_msgs::DisparityImage, dai::ImgFrame> dispPublish(
            stereoQueue,
            pnh,
            std::string("stereo/disparity"),
            std::bind(&dai::rosBridge::DisparityConverter::toRosMsg, &dispConverter, std::placeholders::_1, std::placeholders::_2),
            30,
            disparityCameraInfo,
            "stereo");
        dispPublish.addPublisherCallback();
        if(depthAligned) {
            auto imgQueue = device->getOutputQueue("rgb", 30, false);
            dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
            dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(
                imgQueue,
                pnh,
                std::string("color/image"),
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
                30,
                rgbCameraInfo,
                "color");
            rgbPublish.addPublisherCallback();
            ros::ServiceServer postProcessingService = pnh.advertiseService("setPostProcessing", &DepthPostProcessing::setDepthPostProcessingRequest, &postProcessing);
            ros::spin();
        } else {
            auto leftQueue = device->getOutputQueue("left", 30, false);
            auto rightQueue = device->getOutputQueue("right", 30, false);
            dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftPublish(
                leftQueue,
                pnh,
                leftPubName,
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &converter, std::placeholders::_1, std::placeholders::_2),
                30,
                leftCameraInfo,
                "left");
            dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rightPublish(
                rightQueue,
                pnh,
                rightPubName,
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rightconverter, std::placeholders::_1, std::placeholders::_2),
                30,
                rightCameraInfo,
                "right");
            rightPublish.addPublisherCallback();
            leftPublish.addPublisherCallback();
            ros::ServiceServer postProcessingService = pnh.advertiseService("setPostProcessing", &DepthPostProcessing::setDepthPostProcessingRequest, &postProcessing);
            ros::spin();
        }
        // ros::ServiceServer exposureService = pnh.advertiseService("setExposure", &CameraControl::setRgbExposureRequest, &cameraControl);
        // ros::ServiceServer focusService = pnh.advertiseService("setFocus", &CameraControl::setFocusRequest, &cameraControl);
    }
    

    return 0;
}
