
#include "ros/ros.h"

#include <iostream>
#include <cstdio>
// #include "utility.hpp"
#include <depthai_examples/yolov4_spatial_pipeline.hpp>

#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include <vision_msgs/Detection2DArray.h>

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/SpatialDetectionConverter.hpp>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"


int main(int argc, char** argv){

    ros::init(argc, argv, "mobilenet_node");
    ros::NodeHandle pnh("~");
    
    std::string deviceName;
    std::string camera_param_uri;
    std::string nnPath("/home/luis/Projs_Software/INESC_projects/AOK/YoloV4_Models/depthai_official/tiny-yolo-v4_openvino_2021.2_6shave.blob"); // Set your path for the model here
    int bad_params = 0;

    bad_params += !pnh.getParam("camera_name", deviceName);
    bad_params += !pnh.getParam("camera_param_uri", camera_param_uri);

    if (bad_params > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }

    YoloSpatialDetectionExample detectionPipeline;
    detectionPipeline.initDepthaiDev(nnPath);
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues = detectionPipeline.getExposedImageStreams();
    std::vector<std::shared_ptr<dai::DataOutputQueue>> nNetDataQueues = detectionPipeline.getExposedNnetStreams();

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


    dai::rosBridge::SpatialDetectionConverter detConverter(deviceName + "_rgb_camera_optical_frame", 416, 416, false);
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(nNetDataQueues[0],
                                                                                                         pnh, 
                                                                                                         std::string("color/yolov4_Spatial_detections"),
                                                                                                         std::bind(static_cast<void(dai::rosBridge::SpatialDetectionConverter::*)(std::shared_ptr<dai::SpatialImgDetections>, 
                                                                                                         depthai_ros_msgs::SpatialDetectionArray&)>(&dai::rosBridge::SpatialDetectionConverter::toRosMsg), 
                                                                                                         &detConverter,
                                                                                                         std::placeholders::_1, 
                                                                                                         std::placeholders::_2) , 
                                                                                                         30);


    detectionPublish.startPublisherThread(); 
    rgbPublish.addPubisherCallback(); // addPubisherCallback works only when the dataqueue is non blocking.

    ros::spin();

    return 0;
}
