#include "common.hpp"

using TemporalMode = dai::RawStereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode;
using DecimationMode = dai::RawStereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode;

DepthPostProcessing::DepthPostProcessing(ros_node node) {
    set_parameter("median_enable", _median_enable);
    set_parameter("median_mode", _median_mode);
    set_parameter("speckle_enable", _speckle_enable);
    set_parameter("speckle_range", _speckle_range);
    set_parameter("temporal_enable", _temporal_enable);
    set_parameter("temporal_mode", _temporal_mode);
    set_parameter("temporal_alpha", _temporal_alpha);
    set_parameter("temporal_delta", _temporal_delta);
    set_parameter("spatial_enable", _spatial_enable);
    set_parameter("spatial_radius", _spatial_radius);
    set_parameter("spatial_alpha", _spatial_alpha);
    set_parameter("spatial_delta", _spatial_delta);
    set_parameter("spatial_iterations", _spatial_iterations);
    set_parameter("threshold_enable", _threshold_enable);
    set_parameter("threshold_max", _threshold_max);
    set_parameter("threshold_min", _threshold_min);
    set_parameter("decimation_enable", _decimation_enable);
    set_parameter("decimation_mode", _decimation_mode);
    set_parameter("decimation_factor", _decimation_factor);
}

dai::MedianFilter DepthPostProcessing::getMedianFilter() {
    if(_median_mode == "MEDIAN_OFF") return dai::MedianFilter::MEDIAN_OFF;
    if(_median_mode == "KERNEL_3x3") return dai::MedianFilter::KERNEL_3x3;
    if(_median_mode == "KERNEL_5x5") return dai::MedianFilter::KERNEL_5x5;
    if(_median_mode == "KERNEL_7x7") return dai::MedianFilter::KERNEL_7x7;
    return dai::MedianFilter::MEDIAN_OFF;
}

TemporalMode DepthPostProcessing::getTemporalMode() {
    if(_temporal_mode == "PERSISTENCY_OFF") return TemporalMode::PERSISTENCY_OFF;
    if(_temporal_mode == "VALID_8_OUT_OF_8") return TemporalMode::VALID_8_OUT_OF_8;
    if(_temporal_mode == "VALID_2_IN_LAST_3") return TemporalMode::VALID_2_IN_LAST_3;
    if(_temporal_mode == "VALID_2_IN_LAST_4") return TemporalMode::VALID_2_IN_LAST_4;
    if(_temporal_mode == "VALID_2_OUT_OF_8") return TemporalMode::VALID_2_OUT_OF_8;
    if(_temporal_mode == "VALID_1_IN_LAST_2") return TemporalMode::VALID_1_IN_LAST_2;
    if(_temporal_mode == "VALID_1_IN_LAST_5") return TemporalMode::VALID_1_IN_LAST_5;
    if(_temporal_mode == "VALID_1_IN_LAST_8") return TemporalMode::VALID_1_IN_LAST_8;
    if(_temporal_mode == "PERSISTENCY_INDEFINITELY") return TemporalMode::PERSISTENCY_INDEFINITELY;
    return TemporalMode::PERSISTENCY_OFF;
}

DecimationMode DepthPostProcessing::getDecimationMode() {
    if(_decimation_mode == "PIXEL_SKIPPING") return DecimationMode::PIXEL_SKIPPING;
    if(_decimation_mode == "NON_ZERO_MEDIAN") return DecimationMode::NON_ZERO_MEDIAN;
    if(_decimation_mode == "NON_ZERO_MEAN") return DecimationMode::NON_ZERO_MEAN;
    return DecimationMode::PIXEL_SKIPPING;
}

void DepthPostProcessing::setMedianFilter() {
    if(_median_enable) _stereo->initialConfig.setMedianFilter(getMedianFilter());
}

dai::RawStereoDepthConfig DepthPostProcessing::getFilters(dai::RawStereoDepthConfig config) {
    // auto config = _stereo->initialConfig.get();
    // auto config = dai::StereoDepthConfig();
    if(_speckle_enable) {
        config.postProcessing.speckleFilter.enable = _speckle_enable;
        config.postProcessing.speckleFilter.speckleRange = static_cast<std::uint32_t>(_speckle_range);
    }
    if(_temporal_enable) {
        config.postProcessing.temporalFilter.enable = _temporal_enable;
        config.postProcessing.temporalFilter.alpha = _temporal_alpha;
        config.postProcessing.temporalFilter.delta = static_cast<std::int32_t>(_temporal_delta);
        config.postProcessing.temporalFilter.persistencyMode = getTemporalMode();
    }
    if(_spatial_enable) {
        config.postProcessing.spatialFilter.enable = _spatial_enable;
        config.postProcessing.spatialFilter.holeFillingRadius = static_cast<std::uint8_t>(_spatial_radius);
        config.postProcessing.spatialFilter.alpha = _spatial_alpha;
        config.postProcessing.spatialFilter.delta = static_cast<std::int32_t>(_spatial_delta);
        config.postProcessing.spatialFilter.numIterations = static_cast<std::int32_t>(_spatial_iterations);
    }
    if(_threshold_enable) {
        config.postProcessing.thresholdFilter.minRange = _threshold_min;
        config.postProcessing.thresholdFilter.maxRange = _threshold_max;
    }
    if(_decimation_enable) {
        config.postProcessing.decimationFilter.decimationFactor = static_cast<std::uint32_t>(_decimation_factor);
        config.postProcessing.decimationFilter.decimationMode = getDecimationMode();
    }
    return config;
    // _stereo->initialConfig.set(config);
}

void DepthPostProcessing::setDevice(std::shared_ptr<dai::node::StereoDepth> stereo) {
    _stereo = stereo;
}