
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <cstdio>

#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <vision_msgs/msg/Detection2DArray.hpp>

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/ImgDetectionConverter.hpp>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

dai::Pipeline createPipeline(bool syncNN, std::string nnPath){
    dai::Pipeline pipeline;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    auto detectionNetwork = pipeline.create<dai::node::MobileNetDetectionNetwork>();
    auto nnOut = pipeline.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("preview");
    nnOut->setStreamName("detections");

    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(40);

    // testing MobileNet DetectionNetwork
    detectionNetwork->setConfidenceThreshold(0.5f);
    detectionNetwork->setBlobPath(nnPath);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(detectionNetwork->input);
    if(syncNN) detectionNetwork->passthrough.link(xlinkOut->input);
    else colorCam->preview.link(xlinkOut->input);

    detectionNetwork->out.link(nnOut->input);
    return pipeline;
}

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_node");
    
    std::string deviceName;
    std::string cameraParamUri;
    std::string nnPath(BLOB_PATH);
    bool syncNN;
    int bad_params = 0;

    bad_params += !node->get_parameter("camera_name", deviceName);
    bad_params += !node->get_parameter("camera_param_uri", cameraParamUri);
    bad_params += !node->get_parameter("sync_nn", syncNN);

    if (bad_params > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }

    // Uses the path from param if passed or else uses from BLOB_PATH from CMAKE
    if (node->has_parameter ("nn_path"))
    {
        node->get_parameter("nn_path", nnPath);
    }

    dai::Pipeline pipeline = createPipeline(syncNN, nnPath);
    dai::Device device(pipeline);
    
    std::shared_ptr<dai::DataOutputQueue> previewQueue = device.getOutputQueue("preview", 30, false);
    std::shared_ptr<dai::DataOutputQueue> nNetDataQueue = device.getOutputQueue("detections", 30, false);

    std::string color_uri = cameraParamUri + "/" + "color.yaml";

    dai::rosBridge::ImageConverter rgbConverter(deviceName + "_rgb_camera_optical_frame", false);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(previewQueue,
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


    dai::rosBridge::ImgDetectionConverter detConverter(deviceName + "_rgb_camera_optical_frame", 300, 300, false);
    dai::rosBridge::BridgePublisher<vision_msgs::Detection2DArray, dai::ImgDetections> detectionPublish(nNetDataQueue,
                                                                                                         node, 
                                                                                                         std::string("color/mobilenet_detections"),
                                                                                                         std::bind(static_cast<void(dai::rosBridge::ImgDetectionConverter::*)(std::shared_ptr<dai::ImgDetections>, 
                                                                                                         vision_msgs::Detection2DArray&)>(&dai::rosBridge::ImgDetectionConverter::toRosMsg), 
                                                                                                         &detConverter,
                                                                                                         std::placeholders::_1, 
                                                                                                         std::placeholders::_2), 
                                                                                                         30);

    detectionPublish.startPublisherThread();
    rgbPublish.addPubisherCallback(); // addPubisherCallback works only when the dataqueue is non blocking.

    ros::spin();

    return 0;
}

