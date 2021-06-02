#include <depthai_examples/stereo_from_host.hpp>


void StereoHost::initDepthaiDev(){
    
    // bool withDepth = true;
    bool outputDepth = true;
    bool outputRectified = false;
    bool lrcheck  = false;
    bool extended = false;
    bool subpixel = false;

    /**    xLinkInLeft 
     *                \
     *                 |-> stereo |-> xLinkOutDept
     *                / 
     *    xLinkInRight
     */

    auto xlinkInLeft  = _p.create<dai::node::XLinkIn>();
    auto xlinkInRight = _p.create<dai::node::XLinkIn>();
    auto stereo       = _p.create<dai::node::StereoDepth>();
    auto xoutDepth    = _p.create<dai::node::XLinkOut>();


    xlinkInLeft ->setStreamName("in_left");
    xlinkInRight->setStreamName("in_right");

    xoutDepth->setStreamName("depth");
    
    // StereoDepth
    stereo->setOutputDepth(outputDepth);
    stereo->setOutputRectified(outputRectified);
    stereo->setConfidenceThreshold(200);
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout

    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);
    
    stereo->setInputResolution(1280, 720);
    // Link plugins CAM -> STEREO -> XLINK
    xlinkInLeft->out.link(stereo->left);
    xlinkInRight->out.link(stereo->right);

    stereo->depth.link(xoutDepth->input);

    // CONNECT TO DEVICE
     _dev = std::make_unique<dai::Device>(_p);
     _dev->startPipeline();

    _inImageStreams.push_back(_dev->getInputQueue("in_left"));
    _inImageStreams.push_back(_dev->getInputQueue("in_right"));

    _opImageStreams.push_back(_dev->getOutputQueue("depth", 30, false));
    
}

std::vector<std::shared_ptr<dai::DataOutputQueue>> StereoHost::getExposedOutputImageStreams(){
    return _opImageStreams;
}

std::vector<std::shared_ptr<dai::DataInputQueue>> StereoHost::getExposedInputImageStreams(){
    return _inImageStreams;
}
