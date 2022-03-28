#include "common.hpp"

CameraControl::CameraControl() {
    rgb.name = "rgb";
    stereo.name = "stereo";
}

void CameraControl::setRgbExposure(bool value){
    _exposure_rgb = value;
}

void CameraControl::setExposure() {
    setExposure(stereo);
    if (_exposure_rgb)
        setExposure(rgb);
}

void CameraControl::setExposure(ExposureParameters exposure) {
    auto controlQueue = _device->getInputQueue("control_" + exposure.name);
    auto configQueue = _device->getInputQueue("config_" + exposure.name);
    dai::CameraControl ctrl;
    if(exposure.auto_exposure) {
        ctrl.setAutoExposureEnable();
    } else {
        if(exposure.region.at(2) != 0 || exposure.region.at(3) != 0) {
            dai::ImageManipConfig cfg;
            cfg.setCropRect(exposure.region.at(0), exposure.region.at(1), exposure.region.at(2), exposure.region.at(3));
            configQueue->send(cfg);
        }
        else {
            ctrl.setManualExposure(exposure.time_us, exposure.sensitivity_iso);
        }
    }
    ctrl.setAutoExposureCompensation(exposure.compensation);
    controlQueue->send(ctrl);
}

void CameraControl::setDevice(std::shared_ptr<dai::Device> device) {
    _device = device;
}

dai::CameraControl::AutoFocusMode CameraControl::getFocusMode() {
    if (focus_mode == "AUTO")
        return dai::CameraControl::AutoFocusMode::AUTO;
    if (focus_mode == "CONTINUOUS_PICTURE")
        return dai::CameraControl::AutoFocusMode::CONTINUOUS_PICTURE;
    if (focus_mode == "CONTINUOUS_VIDEO ")
        return dai::CameraControl::AutoFocusMode::CONTINUOUS_VIDEO;
    if (focus_mode == "EDOF")
        return dai::CameraControl::AutoFocusMode::EDOF;
    if (focus_mode == "MACRO")
        return dai::CameraControl::AutoFocusMode::MACRO;
    if (focus_mode == "OFF")
        return dai::CameraControl::AutoFocusMode::OFF;
    return dai::CameraControl::AutoFocusMode::AUTO;
}

void CameraControl::setFocus() {
    dai::CameraControl ctrl;
    auto controlQueue = _device->getInputQueue("control");
    auto focus = getFocusMode();
    ctrl.setAutoFocusMode(focus);
    
    if (focus_mode == "OFF") {
        auto configQueue = _device->getInputQueue("config");
        dai::ImageManipConfig cfg;
        cfg.setCropRect(focus_region.at(0), focus_region.at(1), 0, 0);
        configQueue->send(cfg);
    } else if (focus_region.at(2) > 0 && focus_region.at(3) > 0)
        ctrl.setAutoFocusRegion(focus_region.at(0), focus_region.at(1), focus_region.at(2), focus_region.at(3));
    controlQueue->send(ctrl);
}

req_type CameraControl::setRgbExposureRequest(exp_req_msg request, exp_rep_msg response) {
    rgb.auto_exposure = req_get(auto_exposure);
    rgb.region.at(0) = req_get(exposure_x);
    rgb.region.at(1) = req_get(exposure_y);
    rgb.region.at(2) = req_get(exposure_width);
    rgb.region.at(3) = req_get(exposure_height);
    rgb.compensation = req_get(compensation);
    rgb.time_us = req_get(exposure_time_us);
    rgb.sensitivity_iso = req_get(sensitivity_iso);
    setExposure(rgb);
    rep_get(success) = true;
    bool result = true;
    return (req_type)result;
}

req_type CameraControl::setStereoExposureRequest(exp_req_msg request, exp_rep_msg response) {
    stereo.auto_exposure = req_get(auto_exposure);
    stereo.region.at(0) = req_get(exposure_x);
    stereo.region.at(1) = req_get(exposure_y);
    stereo.region.at(2) = req_get(exposure_width);
    stereo.region.at(3) = req_get(exposure_height);
    stereo.compensation = req_get(compensation);
    stereo.time_us = req_get(exposure_time_us);
    stereo.sensitivity_iso = req_get(sensitivity_iso);
    setExposure(rgb);
    rep_get(success) = true;
    bool result = true;
    return (req_type)result;
}

req_type CameraControl::setFocusRequest(foc_req_msg request, foc_rep_msg response) {
    focus_mode = req_get(focus_mode);
    focus_region.at(0) = req_get(focus_x);
    focus_region.at(1) = req_get(focus_y);
    focus_region.at(2) = req_get(focus_width);
    focus_region.at(3) = req_get(focus_height);
    setFocus();
    rep_get(success) = true;
    bool result = true;
    return (req_type)result;
}