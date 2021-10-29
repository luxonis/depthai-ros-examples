
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <cstdio>
// #include "utility.hpp"
#include <depthai_examples/yolov4_spatial_pipeline.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <vision_msgs/msg/Detection2DArray.h>

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/SpatialDetectionConverter.hpp>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

const std::vector<std::string> label_map = {"person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
             "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
             "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
             "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
             "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
             "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
             "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
             "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
             "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
             "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
             "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
             "teddy bear",     "hair drier", "toothbrush"};

dai::Pipeline createPipeline(bool syncNN, bool subpixel, std::string nnPath){
    dai::Pipeline pipeline;
    auto colorCam = _p.create<dai::node::ColorCamera>();
    auto spatialDetectionNetwork = _p.create<dai::node::YoloSpatialDetectionNetwork>();
    auto monoLeft =  _p.create<dai::node::MonoCamera>();
    auto monoRight = _p.create<dai::node::MonoCamera>();
    auto stereo =    _p.create<dai::node::StereoDepth>();

    // create xlink connections
    auto xoutRgb = _p.create<dai::node::XLinkOut>();
    auto xoutDepth = _p.create<dai::node::XLinkOut>();
    auto xoutNN =  _p.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("preview");
    xoutNN->setStreamName("detections");
    xoutDepth->setStreamName("depth");

    colorCam->setPreviewSize(416, 416);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    /// setting node configs
    stereo->setConfidenceThreshold(255);
    stereo->setSubpixel(subpixel);

    spatialDetectionNetwork->setBlobPath(nnPath);
    spatialDetectionNetwork->setConfidenceThreshold(0.5f);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);

    // yolo specific parameters
    spatialDetectionNetwork->setNumClasses(80);
    spatialDetectionNetwork->setCoordinateSize(4);
    spatialDetectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    spatialDetectionNetwork->setAnchorMasks({{"side13", {3, 4, 5}}, {"side26", {1, 2, 3}}});
    spatialDetectionNetwork->setIouThreshold(0.5f);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(spatialDetectionNetwork->input);
    if(syncNN)
        spatialDetectionNetwork->passthrough.link(xoutRgb->input);
    else
        colorCam->preview.link(xoutRgb->input);

    spatialDetectionNetwork->out.link(xoutNN->input);

    stereo->depth.link(spatialDetectionNetwork->inputDepth);
    spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);
}

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("yolov4_spatial_node");
    
    std::string deviceName;
    std::string camera_param_uri;
    std::string nnPath(BLOB_PATH); // Set your path for the model here
    bool syncNN, subpixel;

    int bad_params = 0;

    bad_params += !node->get_parameter("camera_name", deviceName);
    bad_params += !node->get_parameter("camera_param_uri", camera_param_uri);
    bad_params += !node->get_parameter("sync_nn", syncNN);
    bad_params += !node->get_parameter("subpixel", subpixel);

    if (bad_params > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }

    if (node->has_parameter("nn_path"))
    {
      node->get_parameter("nn_path", nnPath);
    }

    dai::Pipeline pipeline = createPipeline(syncNN, nnPath);
    dai::Device device(pipeline);

    std::string color_uri = camera_param_uri + "/" + "color.yaml";
    std::string stereo_uri = camera_param_uri + "/" + "right.yaml";

    dai::rosBridge::ImageConverter rgbConverter(deviceName + "_rgb_camera_optical_frame", false);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(device.getOutputQueue("preview", 30, false),
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

    dai::rosBridge::SpatialDetectionConverter detConverter(deviceName + "_rgb_camera_optical_frame", 416, 416, false);
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::msg::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(device.getOutputQueue("detections", 30, false),
                                                                                                         node, 
                                                                                                         std::string("color/yolov4_Spatial_detections"),
                                                                                                         std::bind(static_cast<void(dai::rosBridge::SpatialDetectionConverter::*)(std::shared_ptr<dai::SpatialImgDetections>, 
                                                                                                         depthai_ros_msgs::SpatialDetectionArray&)>(&dai::rosBridge::SpatialDetectionConverter::toRosMsg), 
                                                                                                         &detConverter,
                                                                                                         std::placeholders::_1, 
                                                                                                         std::placeholders::_2) , 
                                                                                                         30);

    dai::rosBridge::ImageConverter depthConverter(deviceName + "_right_camera_optical_frame", true);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(device.getOutputQueue("depth", 30, false),
                                                                                     node, 
                                                                                     std::string("stereo/depth"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &depthConverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     stereo_uri,
                                                                                     "stereo");

    depthPublish.addPubisherCallback();

    detectionPublish.addPubisherCallback(); 
    rgbPublish.addPubisherCallback(); // addPubisherCallback works only when the dataqueue is non blocking.

    ros::spin();

    return 0;
}
