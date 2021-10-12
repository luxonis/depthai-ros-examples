#include <chrono>
#include <depthai_bridge/BridgePublisher.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static const std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                                  "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                                  "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};

static std::atomic<bool> fullFrameTracking{false};

void toRosMsg(std::shared_ptr<dai::Tracklets> inData, geometry_msgs::msg::PoseStamped& loc) {
    // int cols = 30, rows = 300;
    auto tracklets = inData->tracklets;
    size_t size = tracklets.size();
    if(size > 0) {
        auto t = tracklets[size - 1];
        // auto roi = t.roi.denormalize(frame.cols, frame.rows);
        // int x1 = roi.topLeft().x;
        // int y1 = roi.topLeft().y;
        // int x2 = roi.bottomRight().x;
        // int y2 = roi.bottomRight().y;
        loc.header.stamp = rclcpp::Clock().now();
        loc.header.frame_id = "OAK-D";
        loc.pose.position.x = t.spatialCoordinates.x / 1000;  // Converting to meters
        loc.pose.position.y = t.spatialCoordinates.y / 1000;  // Converting to meters
        loc.pose.position.z = t.spatialCoordinates.z / 1000;  // Converting to meters
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

    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto trackerOut = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("preview");
    trackerOut->setStreamName("tracklets");

    // Properties
    camRgb->setPreviewSize(300, 300);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

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
    objectTracker->passthroughTrackerFrame.link(xoutRgb->input);
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

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto preview = device.getOutputQueue("preview", 4, false);
    auto trackletsQueue = device.getOutputQueue("tracklets", 4, false);

    dai::rosBridge::BridgePublisher<geometry_msgs::msg::PoseStamped, dai::Tracklets> trackerPublish(
        trackletsQueue, node, std::string("tracked_point"), toRosMsg, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    trackerPublish.addPubisherCallback();
    rclcpp::spin(node);

    /*     while(true) {
            auto imgFrame = preview->get<dai::ImgFrame>();
            auto track = tracklets->get<dai::Tracklets>();

            counter++;
            auto currentTime = steady_clock::now();
            auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
            if(elapsed > seconds(1)) {
                fps = counter / elapsed.count();
                counter = 0;
                startTime = currentTime;
            }

            cv::Mat frame = imgFrame->getCvFrame();
            auto trackletsData = track->tracklets;
            for(auto& t : trackletsData) {
                auto roi = t.roi.denormalize(frame.cols, frame.rows);
                int x1 = roi.topLeft().x;
                int y1 = roi.topLeft().y;
                int x2 = roi.bottomRight().x;
                int y2 = roi.bottomRight().y;

                int labelIndex = t.label;
                std::string labelStr = to_string(labelIndex);
                if(labelIndex < labelMap.size()) {
                    labelStr = labelMap[labelIndex];
                }
                cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

                std::stringstream idStr;
                idStr << "ID: " << t.id;
                cv::putText(frame, idStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
                std::stringstream statusStr;
                statusStr << "Status: " << t.status;
                cv::putText(frame, statusStr.str(), cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

                std::stringstream depthX;
                depthX << "X: " << (int)t.spatialCoordinates.x << " mm";
                cv::putText(frame, depthX.str(), cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
                std::stringstream depthY;
                depthY << "Y: " << (int)t.spatialCoordinates.y << " mm";
                cv::putText(frame, depthY.str(), cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
                std::stringstream depthZ;
                depthZ << "Z: " << (int)t.spatialCoordinates.z << " mm";
                cv::putText(frame, depthZ.str(), cv::Point(x1 + 10, y1 + 95), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

                cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
            }

            std::stringstream fpsStr;
            fpsStr << "NN fps: " << std::fixed << std::setprecision(2) << fps;
            cv::putText(frame, fpsStr.str(), cv::Point(2, imgFrame->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

            cv::imshow("tracker", frame);

            int key = cv::waitKey(1);
            if(key == 'q') {
                return 0;
            }
        } */
    return 0;
}
