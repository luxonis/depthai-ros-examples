
#include <cstdio>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
// #include "utility.hpp"
#include <depthai_examples/stereo_pipeline.hpp>
#include <functional>

#include "sensor_msgs/msg/image.hpp"

// #include <depthai_examples/daiUtility.hpp>
// Inludes common necessary includes for development using depthai library
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

#include "depthai/depthai.hpp"

// using namespace std::placeholders;
int main(int argc, char** argv) {
    std::cout << "Spinning 1 contd... " << std::endl;

    rclcpp::init(argc, argv);

    std::cout << "Spinning 1 contd... " << std::endl;

    auto node = rclcpp::Node::make_shared("stereo_node");

    RCLCPP_INFO(node->get_logger(), "This is my log message");

    std::cout << "Spinning 1 contd... " << std::endl;
    std::string deviceName = "oak-d";
    std::string camera_param_uri = "package://depthai_examples/params/camera";
    // int bad_params = 0;

    // bad_params += !node->get_parameter("camera_name", deviceName);
    // bad_params += !node->get_parameter("camera_param_uri", camera_param_uri);

    // std::cout << "Spinning 1 contd... " << std::endl;
    // if (bad_params > 0)
    // {

    // std::cout << "Spinning 2 contd... " << std::endl;
    //     throw std::runtime_error("Couldn't find one of the parameters");
    // }
    std::cout << "Spinning 2 - contd... " << std::endl;

    StereoExample stero_pipeline;
    stero_pipeline.initDepthaiDev();
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues = stero_pipeline.getExposedImageStreams();

    std::cout << "Spinning 3 contd... " << std::endl;

    // this part would be removed once we have calibration-api
    std::string left_uri = camera_param_uri + "/" + "left.yaml";

    std::string right_uri = camera_param_uri + "/" + "right.yaml";

    std::string stereo_uri = camera_param_uri + "/" + "right.yaml";

    dai::rosBridge::ImageConverter converter(deviceName + "_left_camera_optical_frame", true);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(
        imageDataQueues[0],
        node,
        std::string("left/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &converter, std::placeholders::_1, std::placeholders::_2),
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        left_uri,
        "left");

    // bridgePublish.startPublisherThread();
    leftPublish.addPubisherCallback();

    dai::rosBridge::ImageConverter rightconverter(deviceName + "_right_camera_optical_frame", true);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(
        imageDataQueues[1],
        node,
        std::string("right/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rightconverter, std::placeholders::_1, std::placeholders::_2),
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        right_uri,
        "right");

    rightPublish.addPubisherCallback();

    // dai::rosBridge::ImageConverter depthConverter(deviceName + "_right_camera_optical_frame");
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(
        imageDataQueues[2],
        node,
        std::string("stereo/depth"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                  &rightconverter,  // since the converter has the same frame name
                                    // and image type is also same we can reuse it
                  std::placeholders::_1,
                  std::placeholders::_2),
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        stereo_uri,
        "stereo");

    depthPublish.addPubisherCallback();

    // We can add the rectified frames also similar to these publishers.
    // Left them out so that users can play with it by adding and removing

    std::cout << "Spinning 2contd... " << std::endl;

    rclcpp::spin(node);

    std::cout << "Spinning contd... " << std::endl;
    return 0;
}
