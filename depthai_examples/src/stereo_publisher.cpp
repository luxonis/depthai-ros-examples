
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <cstdio>
// #include "utility.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <depthai_examples/stereo_pipeline.hpp>
#include <functional>

// #include <depthai_examples/daiUtility.hpp>
// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

// using namespace std::placeholders;
int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_node");
    // ros::init(argc, argv, "stereo_node");
    // ros::NodeHandle pnh("~");
    
    std::string deviceName;
    std::string camera_param_uri;
    int bad_params = 0;

    bad_params += !node->get_parameter("camera_name", deviceName);
    bad_params += !node->get_parameter("camera_param_uri", camera_param_uri);

    if (bad_params > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }

    StereoExample stero_pipeline;
    stero_pipeline.initDepthaiDev();
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues = stero_pipeline.getExposedImageStreams();
    
    // std::vector<ros::Publisher> imgPubList;
    // std::vector<std::string> frameNames;
    
    // this part would be removed once we have calibration-api
    std::string left_uri = camera_param_uri +"/" + "left.yaml";
  
    std::string right_uri = camera_param_uri + "/" + "right.yaml";
    
    std::string stereo_uri = camera_param_uri + "/" + "right.yaml";
    

   
    dai::rosBridge::ImageConverter converter(deviceName + "_left_camera_optical_frame", true);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(imageDataQueues[0],
                                                                                     node, 
                                                                                     std::string("left/image"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &converter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                                                                                     left_uri,
                                                                                     "left");

    // bridgePublish.startPublisherThread();
    leftPublish.addPubisherCallback();

    dai::rosBridge::ImageConverter rightconverter(deviceName + "_right_camera_optical_frame", true);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(imageDataQueues[1],
                                                                                     node, 
                                                                                     std::string("right/image"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rightconverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                                                                                     right_uri,
                                                                                     "right");

    rightPublish.addPubisherCallback();

    // dai::rosBridge::ImageConverter depthConverter(deviceName + "_right_camera_optical_frame");
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(imageDataQueues[2],
                                                                                     node, 
                                                                                     std::string("stereo/depth"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rightconverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                                                                                     stereo_uri,
                                                                                     "stereo");

    depthPublish.addPubisherCallback();

    // We can add the rectified frames also similar to these publishers. 
    // Left them out so that users can play with it by adding and removing

    rclcpp::spin(node);
    return 0;
}

