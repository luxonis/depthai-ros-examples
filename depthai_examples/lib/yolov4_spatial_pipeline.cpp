
#include <depthai_examples/yolov4_spatial_pipeline.hpp>
#include "depthai/depthai.hpp"

const std::vector<std::string> YoloSpatialDetectionExample::label_map = {"person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
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


void YoloSpatialDetectionExample::initDepthaiDev(std::string nnPath){

    bool syncNN = true;
    auto colorCam = _p.create<dai::node::ColorCamera>();
    auto spatialDetectionNetwork = _p.create<dai::node::YoloSpatialDetectionNetwork>();
    auto monoLeft =  _p.create<dai::node::MonoCamera>();
    auto monoRight = _p.create<dai::node::MonoCamera>();
    auto stereo =    _p.create<dai::node::StereoDepth>();

    // create xlink connections
    auto xoutRgb = _p.create<dai::node::XLinkOut>();
    auto xoutNN =  _p.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("preview");
    xoutNN->setStreamName("detections");

    colorCam->setPreviewSize(416, 416);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    /// setting node configs
    stereo->setOutputDepth(true);
    stereo->setConfidenceThreshold(255);

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

    _dev = std::make_unique<dai::Device>(_p);
    _dev->startPipeline();

    _opImageStreams.push_back(_dev->getOutputQueue("preview", 30, false));
    _opNNetStreams.push_back(_dev->getOutputQueue("detections", 30, false));
}


std::vector<std::shared_ptr<dai::DataOutputQueue>> YoloSpatialDetectionExample::getExposedImageStreams(){
    return _opImageStreams;
}


std::vector<std::shared_ptr<dai::DataOutputQueue>> YoloSpatialDetectionExample::getExposedNnetStreams(){
    return _opNNetStreams;
}

