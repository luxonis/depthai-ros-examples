#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static const std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                                  "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                                  "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};

static std::atomic<bool> fullFrameTracking{false};

void toRosMsg(std::shared_ptr<dai::Tracklets> inData, geometry_msgs::msg::PoseStamped& loc){
    // int cols = 30, rows = 300;
    auto tracklets = inData->tracklets;
    size_t size = tracklets.size();
    if (size > 0)
    {
        auto t = tracklets[size - 1];
        // auto roi = t.roi.denormalize(frame.cols, frame.rows);
        // int x1 = roi.topLeft().x;
        // int y1 = roi.topLeft().y;
        // int x2 = roi.bottomRight().x;
        // int y2 = roi.bottomRight().y;
        loc.header.stamp = rclcpp::Clock().now();
        loc.header.frame_id = "OAK-D";
        loc.pose.position.x = t.spatialCoordinates.x / 1000; // Converting to meters
        loc.pose.position.y = t.spatialCoordinates.y / 1000; // Converting to meters
        loc.pose.position.z = t.spatialCoordinates.z / 1000; // Converting to meters
        loc.pose.orientation.x = 0;
        loc.pose.orientation.y = 0;
        loc.pose.orientation.z = 0;
        loc.pose.orientation.w = 1;
    }
}

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("dynamic_tracker");
    
    std::string nnPath(BLOB_PATH);

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto spatialDetectionNetwork = pipeline.create<dai::node::MobileNetSpatialDetectionNetwork>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto objectTracker = pipeline.create<dai::node::ObjectTracker>();

    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto trackerOut = pipeline.create<dai::node::XLinkOut>();

    xoutDepth->setStreamName("depth");
    trackerOut->setStreamName("tracklets");
    xoutRight->setStreamName("right");
    // Properties
    camRgb->setPreviewSize(300, 300);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    camRgb->setFps(5);

    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoLeft->setFps(5);
    monoRight->setFps(5);
    
    /// setting node configs
    stereo->initialConfig.setConfidenceThreshold(255);

    spatialDetectionNetwork->setBlobPath(nnPath);
    spatialDetectionNetwork->setConfidenceThreshold(0.5f);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);

    objectTracker->setDetectionLabelsToTrack({15});  // track only person
    // possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS
    objectTracker->setTrackerType(dai::TrackerType::ZERO_TERM_COLOR_HISTOGRAM);
    // take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
    objectTracker->setTrackerIdAssigmentPolicy(dai::TrackerIdAssigmentPolicy::SMALLEST_ID);

    // Linking
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    camRgb->preview.link(spatialDetectionNetwork->input);
    // objectTracker->passthroughTrackerFrame.link(xoutRgb->input);
    objectTracker->out.link(trackerOut->input);

    if(fullFrameTracking) {
        camRgb->setPreviewKeepAspectRatio(false);
        camRgb->video.link(objectTracker->inputTrackerFrame);
        objectTracker->inputTrackerFrame.setBlocking(false);
        // do not block the pipeline if it's too slow on full frame
        objectTracker->inputTrackerFrame.setQueueSize(2);
    } else {
        spatialDetectionNetwork->passthrough.link(objectTracker->inputTrackerFrame);
    }

    spatialDetectionNetwork->passthrough.link(objectTracker->inputDetectionFrame);
    spatialDetectionNetwork->out.link(objectTracker->inputDetections);
    stereo->depth.link(spatialDetectionNetwork->inputDepth);
    stereo->depth.link(xoutDepth->input);
    stereo->syncedRight.link(xoutRight->input);
    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto depthQueue = device.getOutputQueue("depth", 4, false);
    auto rightQueue = device.getOutputQueue("right", 4, false);
    auto trackletsQueue = device.getOutputQueue("tracklets", 4, false);

    std::string camera_param_uri = "package://depthai_examples/params/camera";
    std::string stereo_uri = camera_param_uri + "/" + "right.yaml";
    
    dai::rosBridge::BridgePublisher<geometry_msgs::msg::PoseStamped, dai::Tracklets> trackerPublish(trackletsQueue,
                                                                                                    node, 
                                                                                                    std::string("goal_update"),
                                                                                                    toRosMsg, 
                                                                                                    rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

    trackerPublish.addPubisherCallback();

    dai::rosBridge::ImageConverter depthConverter("oak-d_right_camera_optical_frame", true);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(depthQueue,
                                                                                     node, 
                                                                                     std::string("stereo/depth"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &depthConverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
                                                                                     stereo_uri,
                                                                                     "stereo");
                                                                                     
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(rightQueue,
                                                                                     node, 
                                                                                     std::string("right/image"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &depthConverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
                                                                                     stereo_uri,
                                                                                     "right");
    depthPublish.addPubisherCallback();
    rightPublish.addPubisherCallback();
    rclcpp::spin(node);

    return 0;
}
