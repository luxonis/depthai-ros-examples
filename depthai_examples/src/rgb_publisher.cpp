
#include <cstdio>
#include <iostream>

#include "ros/ros.h"
// #include "utility.hpp"
#include <camera_info_manager/camera_info_manager.h>

#include <depthai_examples/rgb_pipeline.hpp>

#include "sensor_msgs/Image.h"
// #include <depthai_examples/daiUtility.hpp>
// Inludes common necessary includes for development using depthai library
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rgb_node");
    ros::NodeHandle pnh("~");

    std::string deviceName;
    std::string camera_param_uri;
    int bad_params = 0;

    bad_params += !pnh.getParam("camera_name", deviceName);
    bad_params += !pnh.getParam("camera_param_uri", camera_param_uri);

    if(bad_params > 0) {
        throw std::runtime_error("Couldn't find one of the parameters");
    }

    RgbCameraPipelineExample rgbPipeline;
    rgbPipeline.initDepthaiDev();
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues = rgbPipeline.getExposedImageStreams();

    // for (auto op_que : imageDataQueues){
    //     if (op_que->getName().find("video") != std::string::npos){
    //             imgPubList.push_back(pnh.advertise<sensor_msgs::Image>("color/image", 30));
    //             frameNames.push_back(deviceName + "_rgb_camera_optical_frame");
    //     }
    // }
    std::string color_uri = camera_param_uri + "/" + "color.yaml";

    dai::rosBridge::ImageConverter rgbConverter(deviceName + "_rgb_camera_optical_frame", false);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(imageDataQueues[0],
                                                                                  pnh,
                                                                                  std::string("color/image"),
                                                                                  std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                            &rgbConverter,  // since the converter has the same frame name
                                                                                                            // and image type is also same we can reuse it
                                                                                            std::placeholders::_1,
                                                                                            std::placeholders::_2),
                                                                                  30,
                                                                                  color_uri,
                                                                                  "color");

    rgbPublish.startPublisherThread();

    // while(ros::ok()){
    //     for(int i = 0; i < imageDataQueues.size(); ++i){
    //         if(imgPubList[i].getNumSubscribers() == 0) continue;
    //         auto imgData = imageDataQueues[i]->get<dai::ImgFrame>();
    //         // std::cout << "id num ->" << i << imageDataQueues[i]->getName() << std::endl;

    //         sensor_msgs::Image imageMsg;
    //         dai::rosImageBridge(imgData, frameNames[i], imageMsg);
    //         imgPubList[i].publish(imageMsg);
    //     }
    //     ros::spinOnce();
    // }
    ros::spin();

    return 0;
}
