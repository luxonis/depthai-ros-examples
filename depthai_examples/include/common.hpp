#ifndef COMMON_HPP_
#define COMMON_HPP_

#include <iostream>
#include <memory>
#include <string>

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/DisparityConverter.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/ImuConverter.hpp>
#include "depthai/depthai.hpp"

#ifndef IS_ROS2

#include "depthai_examples_interfaces/SetFocus.h"
#include "depthai_examples_interfaces/SetExposure.h"

#include "ros/ros.h"

template <typename T>
static void getParamWithWarning(ros::NodeHandle& pnh, const char* key, T val) {
    bool gotParam = pnh.getParam(key, val);
    if(!gotParam) {
        std::stringstream ss;
        ss << val;
        ROS_WARN("Could not find param '%s' on node '%s'. Defaulting to '%s'", key, pnh.getNamespace().c_str(), ss.str().c_str());
    }
}
#define req_type bool
using exp_req_msg = depthai_examples_interfaces::SetExposure::Request&;
using exp_rep_msg = depthai_examples_interfaces::SetExposure::Response&;
using foc_req_msg = depthai_examples_interfaces::SetFocus::Request&;
using foc_rep_msg = depthai_examples_interfaces::SetFocus::Response&;
#define req_get(x) (request.x)
#define rep_get(x) (response.x)

#else

#include "depthai_examples_interfaces/srv/set_focus.hpp"
#include "depthai_examples_interfaces/srv/set_exposure.hpp"
#define req_type void
using exp_req_msg = const std::shared_ptr<depthai_examples_interfaces::srv::SetExposure::Request>;
using exp_rep_msg = std::shared_ptr<depthai_examples_interfaces::srv::SetExposure::Response>;
using foc_req_msg = const std::shared_ptr<depthai_examples_interfaces::srv::SetFocus::Request>;
using foc_rep_msg = std::shared_ptr<depthai_examples_interfaces::srv::SetFocus::Response>;
#define req_get(x) ((*request).x)
#define rep_get(x) ((*response).x)
#endif

class DepthPostProcessing {
   public:
    dai::MedianFilter getMedianFilter();

    using TemporalMode = dai::RawStereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode;
    TemporalMode getTemporalMode();

    using DecimationMode = dai::RawStereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode;
    DecimationMode getDecimationMode();

    void setDevice(std::shared_ptr<dai::node::StereoDepth> stereo);

    void setMedianFilter();

    void setFilters();

    bool median_enable = false;
    std::string median_mode = "MEDIAN_OFF";
    bool speckle_enable = false;
    int speckle_range = 50;
    bool temporal_enable = false;
    std::string temporal_mode = "PERSISTENCY_OFF";
    float temporal_alpha = 0.4;
    int temporal_delta = 0;
    bool spatial_enable = false;
    int spatial_radius = 2;
    float spatial_alpha = 0.5;
    int spatial_delta = 0;
    int spatial_iterations = 1;
    bool threshold_enable = false;
    int threshold_max = 0;
    int threshold_min = 0;
    bool decimation_enable = false;
    std::string decimation_mode = "NON_ZERO_MEDIAN";
    int decimation_factor = 1;
    private:
    std::shared_ptr<dai::node::StereoDepth> _stereo;
};

struct ExposureParameters{
    int compensation = 0;
    int time_us = 8333;
    int sensitivity_iso = 100;
    bool auto_exposure = true;
    std::array<int, 4> region = {0, 0, 0, 0};
    std::string name;
};

class CameraControl {
    public:
    CameraControl();
    void setDevice(std::shared_ptr<dai::Device> device);
    // Exposure
    ExposureParameters rgb, stereo;
    void setExposure();
    req_type setRgbExposureRequest(exp_req_msg request, exp_rep_msg response);
    req_type setStereoExposureRequest(exp_req_msg request, exp_rep_msg response);
    void setRgbExposure(bool value);
    // Focus
    void setFocus();
    std::string focus_mode = "AUTO";
    std::array<int, 4> focus_region = {0, 0, 0, 0};
    req_type setFocusRequest(foc_req_msg request, foc_rep_msg response);

    private:
    void setExposure(ExposureParameters exposure);
    dai::CameraControl::AutoFocusMode getFocusMode();
    std::shared_ptr<dai::Device> _device;
    bool _exposure_rgb = false;
};

#endif
