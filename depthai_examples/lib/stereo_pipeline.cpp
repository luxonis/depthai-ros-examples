#include <depthai_examples/stereo_pipeline.hpp>


void StereoExampe::initDepthaiDev(bool withDepth){
    

    bool outputRectified = true;
    bool lrcheck  = false;
    bool extended = false;
    bool subpixel = false;

    auto monoLeft    = _p.create<dai::node::MonoCamera>();
    auto monoRight   = _p.create<dai::node::MonoCamera>();
    auto xoutLeft    = _p.create<dai::node::XLinkOut>();
    auto xoutRight   = _p.create<dai::node::XLinkOut>();
    auto stereo      = _p.create<dai::node::StereoDepth>();
    auto xoutDepth   = _p.create<dai::node::XLinkOut>();
    auto xoutRectifL = _p.create<dai::node::XLinkOut>();
    auto xoutRectifR = _p.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    xoutRectifL->setStreamName("rectified_left");
    xoutRectifR->setStreamName("rectified_right");
    if (withDepth) {
        xoutDepth->setStreamName("depth");
    }
    else {
        xoutDepth->setStreamName("disparity");

    }

    // MonoCamera
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    int maxDisp = 96;
    if (extended) maxDisp *= 2;
    if (subpixel) maxDisp *= 32; // 5 bits fractional disparity

    // StereoDepth
    stereo->setConfidenceThreshold(200);
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout

    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    stereo->syncedLeft.link(xoutLeft->input);
    stereo->syncedRight.link(xoutRight->input);
    if(outputRectified)
    {
        stereo->rectifiedLeft.link(xoutRectifL->input);
        stereo->rectifiedRight.link(xoutRectifR->input);
    }

    if(withDepth){
        stereo->depth.link(xoutDepth->input);
    }
    else{
        stereo->disparity.link(xoutDepth->input);
    }


    // CONNECT TO DEVICE
     _dev = std::make_unique<dai::Device>(_p);
     _dev->startPipeline();

     _opImageStreams.push_back(_dev->getOutputQueue("left", 30, false));
     _opImageStreams.push_back(_dev->getOutputQueue("right", 30, false));
     _opImageStreams.push_back(_dev->getOutputQueue("rectified_left", 30, false));
     _opImageStreams.push_back(_dev->getOutputQueue("rectified_right", 30, false));
    if (withDepth) {
        _opImageStreams.push_back(_dev->getOutputQueue("depth", 30, false));
    }else{
        _opImageStreams.push_back(_dev->getOutputQueue("disparity", 30, false));
    }
}

std::vector<std::shared_ptr<dai::DataOutputQueue>> StereoExampe::getExposedImageStreams(){
        return _opImageStreams;
}
