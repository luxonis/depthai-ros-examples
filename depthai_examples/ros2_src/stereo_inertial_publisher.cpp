
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <cstdio>
#include <tuple>
#include "sensor_msgs/msg/imu.h"
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <functional>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImuConverter.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/DisparityConverter.hpp>


class PostProcessing {
    public:
    dai::MedianFilter getMedianFilter() {
        if (this->median_mode == "MEDIAN_OFF")
            return dai::MedianFilter::MEDIAN_OFF;
        if (this->median_mode == "KERNEL_3x3")
            return dai::MedianFilter::KERNEL_3x3;
        if (this->median_mode == "KERNEL_5x5")
            return dai::MedianFilter::KERNEL_5x5;
        if (this->median_mode == "KERNEL_7x7")
            return dai::MedianFilter::KERNEL_7x7;
        return dai::MedianFilter::MEDIAN_OFF;
    }

    using TemporalMode = dai::RawStereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode;
    TemporalMode getTemporalMode() {
        if (this->temporal_mode == "PERSISTENCY_OFF")
            return TemporalMode::PERSISTENCY_OFF;
        if (this->temporal_mode == "VALID_8_OUT_OF_8")
            return TemporalMode::VALID_8_OUT_OF_8;
        if (this->temporal_mode == "VALID_2_IN_LAST_3")
            return TemporalMode::VALID_2_IN_LAST_3;
        if (this->temporal_mode == "VALID_2_IN_LAST_4")
            return TemporalMode::VALID_2_IN_LAST_4;
        if (this->temporal_mode == "VALID_2_OUT_OF_8")
            return TemporalMode::VALID_2_OUT_OF_8;        
        if (this->temporal_mode == "VALID_1_IN_LAST_2")
            return TemporalMode::VALID_1_IN_LAST_2;
        if (this->temporal_mode == "VALID_1_IN_LAST_5")
            return TemporalMode::VALID_1_IN_LAST_5;
        if (this->temporal_mode == "VALID_1_IN_LAST_8")
            return TemporalMode::VALID_1_IN_LAST_8;
        if (this->temporal_mode == "PERSISTENCY_INDEFINITELY")
            return TemporalMode::PERSISTENCY_INDEFINITELY;
        return TemporalMode::PERSISTENCY_OFF;
    }

    using DecimationMode = dai::RawStereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode;
    DecimationMode getDecimationMode() {
        if (this->decimation_mode == "PIXEL_SKIPPING")
            return DecimationMode::PIXEL_SKIPPING;
        if (this->decimation_mode == "NON_ZERO_MEDIAN")
            return DecimationMode::NON_ZERO_MEDIAN;
        if (this->decimation_mode == "NON_ZERO_MEAN")
            return DecimationMode::NON_ZERO_MEAN;
        return DecimationMode::PIXEL_SKIPPING;
    }

    bool median_enable          = false;
    std::string median_mode     = "MEDIAN_OFF";
    bool speckle_enable         = false;
    int speckle_range           = 50;
    bool temporal_enable        = false;
    std::string temporal_mode   = "PERSISTENCY_OFF";
    double temporal_alpha       = 0.4;
    int temporal_delta          = 0;
    bool spatial_enable         = false;
    int spatial_radius          = 2;
    double spatial_alpha        = 0.5;
    int spatial_delta           = 0;
    int spatial_iterations      = 1;
    bool threshold_enable       = false;
    int threshold_max           = 0;
    int threshold_min           = 0;
    bool decimation_enable      = false;
    std::string decimation_mode = "NON_ZERO_MEDIAN";
    int decimation_factor       = 1;
};

std::tuple<dai::Pipeline, int, int> createPipeline(bool enableDepth, bool lrcheck, bool extended, bool subpixel, bool rectify, bool depth_aligned, int stereo_fps, int confidence, int LRchecktresh, std::string resolution, PostProcessing postProcessing){
    dai::Pipeline pipeline;

    auto monoLeft  = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo    = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto imu       = pipeline.create<dai::node::IMU>();
    auto xoutImu   = pipeline.create<dai::node::XLinkOut>();

    if (enableDepth) {
        xoutDepth->setStreamName("depth");
    }
    else {
        xoutDepth->setStreamName("disparity");
    }

    xoutImu->setStreamName("imu");

    int width, height;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution; 
    if(resolution == "720p"){
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P; 
        width  = 1280;
        height = 720;
    }else if(resolution == "400p" ){
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P; 
        width  = 640;
        height = 400;
    }else if(resolution == "800p" ){
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P; 
        width  = 1280;
        height = 800;
    }else if(resolution == "480p" ){
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P; 
        width  = 640;
        height = 480;
    }else{
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                    "Invalid parameter. -> monoResolution: %s", resolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }
    // MonoCamera
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoLeft->setFps(stereo_fps);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoRight->setFps(stereo_fps);

    // StereoDepth
    if (postProcessing.median_enable)
        stereo->initialConfig.setMedianFilter(postProcessing.getMedianFilter());

    stereo->initialConfig.setConfidenceThreshold(confidence); //Known to be best
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh); //Known to be best
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);
    auto config = stereo->initialConfig.get();
    if (postProcessing.speckle_enable) {
        config.postProcessing.speckleFilter.enable = postProcessing.speckle_enable;
        config.postProcessing.speckleFilter.speckleRange = postProcessing.speckle_range;
    }
    if (postProcessing.temporal_enable) {
        config.postProcessing.temporalFilter.enable = postProcessing.temporal_enable;
        config.postProcessing.temporalFilter.alpha = postProcessing.temporal_alpha;
        config.postProcessing.temporalFilter.delta = postProcessing.temporal_delta;
        config.postProcessing.temporalFilter.persistencyMode = postProcessing.getTemporalMode();
    }
    if (postProcessing.spatial_enable) {
        config.postProcessing.spatialFilter.enable = postProcessing.spatial_enable;
        config.postProcessing.spatialFilter.holeFillingRadius = postProcessing.spatial_radius;
        config.postProcessing.spatialFilter.alpha = postProcessing.spatial_alpha;
        config.postProcessing.spatialFilter.delta = postProcessing.spatial_delta;
        config.postProcessing.spatialFilter.numIterations = postProcessing.spatial_iterations;
    }
    if (postProcessing.threshold_enable) {
        config.postProcessing.thresholdFilter.minRange = postProcessing.threshold_min;
        config.postProcessing.thresholdFilter.maxRange = postProcessing.threshold_max;
    }
    if (postProcessing.decimation_enable) {
        config.postProcessing.decimationFilter.decimationFactor = postProcessing.decimation_factor;
        config.postProcessing.decimationFilter.decimationMode = postProcessing.getDecimationMode();
    }
    stereo->initialConfig.set(config);
    if(enableDepth && depth_aligned) stereo->setDepthAlign(dai::CameraBoardSocket::RGB);

    //Imu
    imu->enableIMUSensor({dai::IMUSensor::ROTATION_VECTOR, dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 400);
    imu->setMaxBatchReports(1); // Get one message only for now.

    if(enableDepth && depth_aligned){
        // RGB image
        auto camRgb               = pipeline.create<dai::node::ColorCamera>();
        auto xoutRgb              = pipeline.create<dai::node::XLinkOut>();
        xoutRgb->setStreamName("rgb");
        camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
        camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        // the ColorCamera is downscaled from 1080p to 720p.
        // Otherwise, the aligned depth is automatically upscaled to 1080p
        if (height < 720){
            camRgb->setIspScale(1, 3);            
        }
        else{
            camRgb->setIspScale(2, 3);
        }        // For now, RGB needs fixed focus to properly align with depth.
        // This value was used during calibration
        camRgb->initialControl.setManualFocus(135);
        camRgb->isp.link(xoutRgb->input);
    }else{
        // Stereo imges
        auto xoutLeft             = pipeline.create<dai::node::XLinkOut>();
        auto xoutRight            = pipeline.create<dai::node::XLinkOut>();
        // XLinkOut
        xoutLeft->setStreamName("left");
        xoutRight->setStreamName("right");
        if(rectify){
            stereo->rectifiedLeft.link(xoutLeft->input);
            stereo->rectifiedRight.link(xoutRight->input);     
        }else{
            stereo->syncedLeft.link(xoutLeft->input);
            stereo->syncedRight.link(xoutRight->input);
        }
    }

    // Link plugins CAM -> STEREO -> XLINK

    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    if(enableDepth){
        stereo->depth.link(xoutDepth->input);
    }
    else{
        stereo->disparity.link(xoutDepth->input);
    }

    imu->out.link(xoutImu->input);

    return std::make_tuple(pipeline, width, height);
}


int main(int argc, char** argv){
    
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_inertial_node");

    std::string tfPrefix, mode, monoResolution;
    int badParams = 0, stereo_fps, confidence, LRchecktresh;
    bool lrcheck, extended, subpixel, enableDepth, rectify, depth_aligned;

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("mode", "depth");
    node->declare_parameter("lrcheck",  true);
    node->declare_parameter("extended",  false);
    node->declare_parameter("subpixel",  true);
    node->declare_parameter("rectify",  false);
    node->declare_parameter("depth_aligned",  false);
    node->declare_parameter("stereo_fps",  30);
    node->declare_parameter("confidence",  200);
    node->declare_parameter("LRchecktresh",  5);
    node->declare_parameter("monoResolution",  "720p");

    PostProcessing postProcessing;
    node->declare_parameter("median_enable",        postProcessing.median_enable);
    node->declare_parameter("median_mode",          postProcessing.median_mode);
    node->declare_parameter("speckle_enable",       postProcessing.speckle_enable);
    node->declare_parameter("speckle_range",        postProcessing.speckle_range);
    node->declare_parameter("temporal_enable",      postProcessing.temporal_enable);
    node->declare_parameter("temporal_mode",        postProcessing.temporal_mode);
    node->declare_parameter("temporal_alpha",       postProcessing.temporal_alpha);
    node->declare_parameter("temporal_delta",       postProcessing.temporal_delta);
    node->declare_parameter("spatial_enable",       postProcessing.spatial_enable);
    node->declare_parameter("spatial_radius",       postProcessing.spatial_radius);
    node->declare_parameter("spatial_alpha",        postProcessing.spatial_alpha);
    node->declare_parameter("spatial_delta",        postProcessing.spatial_delta);
    node->declare_parameter("spatial_iterations",   postProcessing.spatial_iterations);
    node->declare_parameter("threshold_enable",     postProcessing.threshold_enable);
    node->declare_parameter("threshold_max",        postProcessing.threshold_max);
    node->declare_parameter("threshold_min",        postProcessing.threshold_min);
    node->declare_parameter("decimation_enable",    postProcessing.decimation_enable);
    node->declare_parameter("decimation_mode",      postProcessing.decimation_mode);
    node->declare_parameter("decimation_factor",    postProcessing.decimation_factor);

    node->get_parameter("tf_prefix",     tfPrefix);
    node->get_parameter("mode",          mode);
    node->get_parameter("lrcheck",       lrcheck);
    node->get_parameter("extended",      extended);
    node->get_parameter("subpixel",      subpixel);
    node->get_parameter("rectify",       rectify);
    node->get_parameter("depth_aligned", depth_aligned);
    node->get_parameter("stereo_fps",    stereo_fps);
    node->get_parameter("confidence",    confidence);
    node->get_parameter("LRchecktresh",  LRchecktresh);
    node->get_parameter("monoResolution", monoResolution);

    node->get_parameter("median_enable",        postProcessing.median_enable);
    node->get_parameter("median_mode",          postProcessing.median_mode);
    node->get_parameter("speckle_enable",       postProcessing.speckle_enable);
    node->get_parameter("speckle_range",        postProcessing.speckle_range);
    node->get_parameter("temporal_enable",      postProcessing.temporal_enable);
    node->get_parameter("temporal_mode",        postProcessing.temporal_mode);
    node->get_parameter("temporal_alpha",       postProcessing.temporal_alpha);
    node->get_parameter("temporal_delta",       postProcessing.temporal_delta);
    node->get_parameter("spatial_enable",       postProcessing.spatial_enable);
    node->get_parameter("spatial_radius",       postProcessing.spatial_radius);
    node->get_parameter("spatial_alpha",        postProcessing.spatial_alpha);
    node->get_parameter("spatial_delta",        postProcessing.spatial_delta);
    node->get_parameter("spatial_iterations",   postProcessing.spatial_iterations);
    node->get_parameter("threshold_enable",     postProcessing.threshold_enable);
    node->get_parameter("threshold_max",        postProcessing.threshold_max);
    node->get_parameter("threshold_min",        postProcessing.threshold_min);
    node->get_parameter("decimation_enable",    postProcessing.decimation_enable);
    node->get_parameter("decimation_mode",      postProcessing.decimation_mode);
    node->get_parameter("decimation_factor",    postProcessing.decimation_factor);

    if(mode == "depth"){
        enableDepth = true;
    }
    else{
        enableDepth = false;
    }

    dai::Pipeline pipeline;
    int monoWidth, monoHeight;
    std::tie(pipeline, monoWidth, monoHeight) = createPipeline(enableDepth, lrcheck, extended, subpixel, rectify, depth_aligned, stereo_fps, confidence, LRchecktresh, monoResolution, postProcessing);

    dai::Device device(pipeline);

    std::shared_ptr<dai::DataOutputQueue> stereoQueue;
    if (enableDepth) {
        stereoQueue = device.getOutputQueue("depth", 30, false);
    }else{
        stereoQueue = device.getOutputQueue("disparity", 30, false);
    }
    auto imuQueue = device.getOutputQueue("imu",30,false);

    auto calibrationHandler = device.readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if (monoHeight > 480 && boardName == "OAK-D-LITE") {
        monoWidth = 640;
        monoHeight = 480;
    }
    
    dai::rosBridge::ImageConverter converter(tfPrefix + "_left_camera_optical_frame", true);
    dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
    auto leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, monoWidth, monoHeight); 
    auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, monoWidth, monoHeight); 
    const std::string leftPubName = rectify?std::string("left/image_rect"):std::string("left/image_raw");
    const std::string rightPubName = rectify?std::string("right/image_rect"):std::string("right/image_raw");

    dai::rosBridge::ImuConverter imuConverter(tfPrefix +"_imu_frame");

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData> ImuPublish(imuQueue,
                                                                                     node, 
                                                                                     std::string("imu"),
                                                                                     std::bind(&dai::rosBridge::ImuConverter::toRosMsg, 
                                                                                     &imuConverter, 
                                                                                     std::placeholders::_1,
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     "",
                                                                                     "imu");

    ImuPublish.addPublisherCallback();
    int colorWidth = 1280, colorHeight = 720;
    if (monoHeight < 720) {
        colorWidth = 640; 
        colorHeight = 360;
    }
    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, colorWidth, colorHeight);
    
     if(enableDepth){
        auto depthCameraInfo = depth_aligned ? rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, colorWidth, colorHeight) : rightCameraInfo;
        auto depthconverter = depth_aligned ? rgbConverter : rightconverter;
        dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(stereoQueue,
                                                                                     node, 
                                                                                     std::string("stereo/depth"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &depthconverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     depthCameraInfo,
                                                                                     "stereo");
        depthPublish.addPublisherCallback();
        
        if(depth_aligned){
            auto imgQueue = device.getOutputQueue("rgb", 30, false);
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(imgQueue,
                                                                                        node, 
                                                                                        std::string("color/image"),
                                                                                        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                        &rgbConverter,
                                                                                        std::placeholders::_1, 
                                                                                        std::placeholders::_2) , 
                                                                                        30,
                                                                                        rgbCameraInfo,
                                                                                        "color");
            rgbPublish.addPublisherCallback();
            rclcpp::spin(node);
        }
        else {
            auto leftQueue = device.getOutputQueue("left", 30, false);
            auto rightQueue = device.getOutputQueue("right", 30, false);
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(leftQueue,
                                                                                            node, 
                                                                                            leftPubName,
                                                                                            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                            &converter, 
                                                                                            std::placeholders::_1, 
                                                                                            std::placeholders::_2) , 
                                                                                            30,
                                                                                            leftCameraInfo,
                                                                                            "left");
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(rightQueue,
                                                                                            node, 
                                                                                            rightPubName,
                                                                                            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                            &rightconverter, 
                                                                                            std::placeholders::_1, 
                                                                                            std::placeholders::_2) , 
                                                                                            30,
                                                                                            rightCameraInfo,
                                                                                            "right");  
            rightPublish.addPublisherCallback();
            leftPublish.addPublisherCallback();
            rclcpp::spin(node);
        }
    }
    else{
        std::string tfSuffix = depth_aligned ? "_rgb_camera_optical_frame" : "_right_camera_optical_frame";
        dai::rosBridge::DisparityConverter dispConverter(tfPrefix + tfSuffix , 880, 7.5, 20, 2000); // TODO(sachin): undo hardcoding of baseline
        auto disparityCameraInfo = depth_aligned ? rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, 1280, 720) : rightCameraInfo;
        auto depthconverter = depth_aligned ? rgbConverter : rightconverter;
        dai::rosBridge::BridgePublisher<stereo_msgs::msg::DisparityImage, dai::ImgFrame> dispPublish(stereoQueue,
                                                                                     node, 
                                                                                     std::string("stereo/disparity"),
                                                                                     std::bind(&dai::rosBridge::DisparityConverter::toRosMsg, 
                                                                                     &dispConverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     disparityCameraInfo,
                                                                                     "stereo");
        dispPublish.addPublisherCallback();
        if(depth_aligned){
            auto imgQueue = device.getOutputQueue("rgb", 30, false);
            dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(imgQueue,
                                                                                        node, 
                                                                                        std::string("color/image"),
                                                                                        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                        &rgbConverter,
                                                                                        std::placeholders::_1, 
                                                                                        std::placeholders::_2) , 
                                                                                        30,
                                                                                        rgbCameraInfo,
                                                                                        "color");
            rgbPublish.addPublisherCallback();
            rclcpp::spin(node);
        }
        else {
            auto leftQueue = device.getOutputQueue("left", 30, false);
            auto rightQueue = device.getOutputQueue("right", 30, false);
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(leftQueue,
                                                                                            node, 
                                                                                            leftPubName,
                                                                                            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                            &converter, 
                                                                                            std::placeholders::_1, 
                                                                                            std::placeholders::_2) , 
                                                                                            30,
                                                                                            leftCameraInfo,
                                                                                            "left");
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(rightQueue,
                                                                                            node, 
                                                                                            rightPubName,
                                                                                            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                            &rightconverter, 
                                                                                            std::placeholders::_1, 
                                                                                            std::placeholders::_2) , 
                                                                                            30,
                                                                                            rightCameraInfo,
                                                                                            "right");  
            rightPublish.addPublisherCallback();
            leftPublish.addPublisherCallback();
            rclcpp::spin(node);
        }
    }
    
    return 0;
}
