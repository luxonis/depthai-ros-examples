#include "common.hpp"

using TemporalMode = dai::RawStereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode;
using DecimationMode = dai::RawStereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode;

dai::MedianFilter DepthPostProcessing::getMedianFilter() {
    if(this->median_mode == "MEDIAN_OFF") return dai::MedianFilter::MEDIAN_OFF;
    if(this->median_mode == "KERNEL_3x3") return dai::MedianFilter::KERNEL_3x3;
    if(this->median_mode == "KERNEL_5x5") return dai::MedianFilter::KERNEL_5x5;
    if(this->median_mode == "KERNEL_7x7") return dai::MedianFilter::KERNEL_7x7;
    return dai::MedianFilter::MEDIAN_OFF;
}

TemporalMode DepthPostProcessing::getTemporalMode() {
    if(this->temporal_mode == "PERSISTENCY_OFF") return TemporalMode::PERSISTENCY_OFF;
    if(this->temporal_mode == "VALID_8_OUT_OF_8") return TemporalMode::VALID_8_OUT_OF_8;
    if(this->temporal_mode == "VALID_2_IN_LAST_3") return TemporalMode::VALID_2_IN_LAST_3;
    if(this->temporal_mode == "VALID_2_IN_LAST_4") return TemporalMode::VALID_2_IN_LAST_4;
    if(this->temporal_mode == "VALID_2_OUT_OF_8") return TemporalMode::VALID_2_OUT_OF_8;
    if(this->temporal_mode == "VALID_1_IN_LAST_2") return TemporalMode::VALID_1_IN_LAST_2;
    if(this->temporal_mode == "VALID_1_IN_LAST_5") return TemporalMode::VALID_1_IN_LAST_5;
    if(this->temporal_mode == "VALID_1_IN_LAST_8") return TemporalMode::VALID_1_IN_LAST_8;
    if(this->temporal_mode == "PERSISTENCY_INDEFINITELY") return TemporalMode::PERSISTENCY_INDEFINITELY;
    return TemporalMode::PERSISTENCY_OFF;
}

DecimationMode DepthPostProcessing::getDecimationMode() {
    if(this->decimation_mode == "PIXEL_SKIPPING") return DecimationMode::PIXEL_SKIPPING;
    if(this->decimation_mode == "NON_ZERO_MEDIAN") return DecimationMode::NON_ZERO_MEDIAN;
    if(this->decimation_mode == "NON_ZERO_MEAN") return DecimationMode::NON_ZERO_MEAN;
    return DecimationMode::PIXEL_SKIPPING;
}

void DepthPostProcessing::setMedianFilter() {
    if(this->median_enable) _stereo->initialConfig.setMedianFilter(this->getMedianFilter());
}

void DepthPostProcessing::setFilters() {
    auto config = _stereo->initialConfig.get();
    if(this->speckle_enable) {
        config.postProcessing.speckleFilter.enable = this->speckle_enable;
        config.postProcessing.speckleFilter.speckleRange = static_cast<std::uint32_t>(this->speckle_range);
    }
    if(this->temporal_enable) {
        config.postProcessing.temporalFilter.enable = this->temporal_enable;
        config.postProcessing.temporalFilter.alpha = this->temporal_alpha;
        config.postProcessing.temporalFilter.delta = static_cast<std::int32_t>(this->temporal_delta);
        config.postProcessing.temporalFilter.persistencyMode = this->getTemporalMode();
    }
    if(this->spatial_enable) {
        config.postProcessing.spatialFilter.enable = this->spatial_enable;
        config.postProcessing.spatialFilter.holeFillingRadius = static_cast<std::uint8_t>(this->spatial_radius);
        config.postProcessing.spatialFilter.alpha = this->spatial_alpha;
        config.postProcessing.spatialFilter.delta = static_cast<std::int32_t>(this->spatial_delta);
        config.postProcessing.spatialFilter.numIterations = static_cast<std::int32_t>(this->spatial_iterations);
    }
    if(this->threshold_enable) {
        config.postProcessing.thresholdFilter.minRange = this->threshold_min;
        config.postProcessing.thresholdFilter.maxRange = this->threshold_max;
    }
    if(this->decimation_enable) {
        config.postProcessing.decimationFilter.decimationFactor = static_cast<std::uint32_t>(this->decimation_factor);
        config.postProcessing.decimationFilter.decimationMode = this->getDecimationMode();
    }
    _stereo->initialConfig.set(config);
}

void DepthPostProcessing::setDevice(std::shared_ptr<dai::node::StereoDepth> stereo) {
    _stereo = stereo;
}