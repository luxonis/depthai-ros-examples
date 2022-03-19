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

class CameraControl {
    public:
    void setDevice(std::shared_ptr<dai::Device> device);
    void setExposure();
    bool auto_exposure = true;
    std::array<int, 4> exposure_region = {0, 0, 0, 0};
    int compensation = 0;
    int exposure_time_us = 8333;
    int sensitivity_iso = 100;
    private:
    std::shared_ptr<dai::Device> _device;
};

class FocusSettings {
    public:
    void setDevice(std::shared_ptr<dai::Device> device);
    void setFocus();
    std::string focus_mode = "AUTO";
    std::array<int, 4> focus_region = {0, 0, 0, 0};
    private:
    dai::CameraControl::AutoFocusMode getFocusMode();
    std::shared_ptr<dai::Device> _device;
};

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

bool setExposureRequest(CameraControl& camera,
        depthai_examples_interfaces::SetExposure::Request  &request,
        depthai_examples_interfaces::SetExposure::Response &response);
#else
#include "depthai_examples_interfaces/srv/set_focus.hpp"
#include "depthai_examples_interfaces/srv/set_exposure.hpp"
void setExposureRequest(CameraControl& camera,
    const std::shared_ptr<depthai_examples_interfaces::srv::SetExposure::Request> request,
    std::shared_ptr<depthai_examples_interfaces::srv::SetExposure::Response> response);

void setFocusRequest(FocusSettings& focus,
    const std::shared_ptr<depthai_examples_interfaces::srv::SetFocus::Request> request,
    std::shared_ptr<depthai_examples_interfaces::srv::SetFocus::Response> response);
#endif

#endif
