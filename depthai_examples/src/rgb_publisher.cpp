
#include "ros/ros.h"

#include <iostream>
#include <cstdio>
// #include "utility.hpp"
#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include <depthai_examples/rgb_pipeline.hpp>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>


int main(int argc, char** argv){

    ros::init(argc, argv, "rgb_node");
    ros::NodeHandle pnh("~");
    
    std::string deviceName;
    std::string camera_param_uri;
    int bad_params = 0;

    bad_params += !pnh.getParam("camera_name", deviceName);
    bad_params += !pnh.getParam("camera_param_uri", camera_param_uri);

    if (bad_params > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }
    
    RgbCameraPipelineExample rgbPipeline;
    rgbPipeline.initDepthaiDev();
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues = rgbPipeline.getExposedImageStreams();
    
    std::string color_uri = camera_param_uri + "/" + "color.yaml";

    dai::rosBridge::ImageConverter rgbConverter(deviceName + "_rgb_camera_optical_frame", false);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(imageDataQueues[0],
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

    rgbPublish.startPublisherThread();
    ros::spin();

    return 0;
}

