#include "common.hpp"

void CameraControl::setExposure() {
    auto controlQueue = _device->getInputQueue("control");
    auto configQueue = _device->getInputQueue("config");
    dai::CameraControl ctrl;
    if(auto_exposure) {
        ctrl.setAutoExposureEnable();
    } else {
        if(exposure_region.at(2) != 0 || exposure_region.at(3) != 0) {
            dai::ImageManipConfig cfg;
            cfg.setCropRect(exposure_region.at(0), exposure_region.at(1), exposure_region.at(2), exposure_region.at(3));
            configQueue->send(cfg);
        }
        else {
            ctrl.setManualExposure(exposure_time_us, sensitivity_iso);
        }
    }
    ctrl.setAutoExposureCompensation(compensation);
    controlQueue->send(ctrl);
}

void CameraControl::setDevice(std::shared_ptr<dai::Device> device) {
    _device = device;
}

#ifdef IS_ROS2
void setExposureRequest(CameraControl& camera,
    const std::shared_ptr<depthai_examples_interfaces::srv::SetExposure::Request> request,
    std::shared_ptr<depthai_examples_interfaces::srv::SetExposure::Response> response) {
        camera.auto_exposure = request->auto_exposure;
        camera.exposure_region.at(0) = request->exposure_x;
        camera.exposure_region.at(1) = request->exposure_y;
        camera.exposure_region.at(2) = request->exposure_width;
        camera.exposure_region.at(3) = request->exposure_height;
        camera.compensation = request->compensation;
        camera.exposure_time_us = request->exposure_time_us;
        camera.sensitivity_iso = request->sensitivity_iso;
        camera.setExposure();
        response->success = true;
        return;
}
#else
bool setExposureRequest(CameraControl& camera,
        depthai_examples_interfaces::SetExposure::Request  &request,
        depthai_examples_interfaces::SetExposure::Response &response) {
        camera.auto_exposure = request.auto_exposure;
        camera.exposure_region.at(0) = request.exposure_x;
        camera.exposure_region.at(1) = request.exposure_y;
        camera.exposure_region.at(2) = request.exposure_width;
        camera.exposure_region.at(3) = request.exposure_height;
        camera.compensation = request.compensation;
        camera.exposure_time_us = request.exposure_time_us;
        camera.sensitivity_iso = request.sensitivity_iso;
        camera.setExposure();
        response.success = true;
        return true;
}
#endif
