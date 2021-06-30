

#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <cstdio>
// #include "utility.hpp"
#include <depthai_examples/nn_pipeline.hpp>

#include "sensor_msgs/msg/image.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/ImgDetectionConverter.hpp>

#include <vision_msgs/msg/detection2_d_array.hpp>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"


int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_node");
    

    std::string nnPath(BLOB_PATH);
    std::string deviceName = "oak-d";
    std::string camera_param_uri = "package://depthai_examples/params/camera";

    MobileNetDetectionExample detectionPipeline;
    detectionPipeline.initDepthaiDev(nnPath);
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues = detectionPipeline.getExposedImageStreams();
    std::vector<std::shared_ptr<dai::DataOutputQueue>> nNetDataQueues = detectionPipeline.getExposedNnetStreams();;

    std::string color_uri = camera_param_uri + "/" + "color.yaml";

    dai::rosBridge::ImageConverter rgbConverter("map", false);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(imageDataQueues[0],
                                                                                     node, 
                                                                                     std::string("color/image"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rgbConverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                                                                                     color_uri,
                                                                                     "color");


    dai::rosBridge::ImgDetectionConverter detConverter("map", 300, 300, false);
    dai::rosBridge::BridgePublisher<vision_msgs::msg::Detection2DArray, dai::ImgDetections> detectionPublish(nNetDataQueues[0],
                                                                                     node, 
                                                                                     std::string("color/mobilenet_detections"),
                                                                                     std::bind(static_cast<void(dai::rosBridge::ImgDetectionConverter::*)(std::shared_ptr<dai::ImgDetections>, 
                                                                                     vision_msgs::msg::Detection2DArray&)>(&dai::rosBridge::ImgDetectionConverter::toRosMsg),
                                                                                     &detConverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    // rgbPublish.startPublisherThread();
    detectionPublish.startPublisherThread(); // addPubisherCallback works only when the dataqueue is non blocking.
    rgbPublish.addPubisherCallback();
    // detectionPublish.addPubisherCallback();

    rclcpp::spin(node);

    return 0;
}

