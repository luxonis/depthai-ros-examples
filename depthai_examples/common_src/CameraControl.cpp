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
