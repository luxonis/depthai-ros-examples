#include "common.hpp"
#include "depthai_examples_interfaces/srv/set_focus.hpp"
#include "depthai_examples_interfaces/srv/set_exposure.hpp"

void FocusSettings::setFocus() {
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

dai::CameraControl::AutoFocusMode FocusSettings::getFocusMode() {
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

void FocusSettings::setDevice(std::shared_ptr<dai::Device> device) {
    _device = device;
}

void FocusSettings::setFocusRequest(
    const std::shared_ptr<depthai_examples_interfaces::srv::SetFocus::Request> request,
    std::shared_ptr<depthai_examples_interfaces::srv::SetFocus::Response> response) {
        focus_mode = request->focus_mode;
        focus_region.at(0) = request->focus_x;
        focus_region.at(1) = request->focus_y;
        focus_region.at(2) = request->focus_width;
        focus_region.at(3) = request->focus_height;
        setFocus();
        response->success = true;
        return;
}