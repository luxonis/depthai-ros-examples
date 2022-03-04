#include "common.hpp"

void ExposureSettings::setExposure(dai::Device& device) {
    auto controlQueue = device.getInputQueue("control");
    auto configQueue = device.getInputQueue("config");
    dai::CameraControl ctrl;
    if(auto_exposure) {
        ctrl.setAutoExposureEnable();
    } else {
        ctrl.setManualExposure(exposure_time_us, sensitivity_iso);
        if(exposure_region.at(2) != 0 || exposure_region.at(3) != 0) {
            dai::ImageManipConfig cfg;
            cfg.setCropRect(exposure_region.at(0), exposure_region.at(1), exposure_region.at(2), exposure_region.at(3));
            configQueue->send(cfg);
        }
    }
    controlQueue->send(ctrl);
}
