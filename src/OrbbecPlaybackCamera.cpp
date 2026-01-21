#include "OrbbecPlaybackCamera.hpp"
#include "OrbbecConfig.hpp"

OrbbecPlaybackCamera::OrbbecPlaybackCamera(std::shared_ptr<ob::PlaybackDevice> camera, OrbbecCaptureConfig& config, int cameraIndex, std::string filename)
:   OrbbecBaseCamera("cwipc_orbbec: OrbbecPlaybackCamera", camera, config, cameraIndex),
    playback_filename(filename)
{
}

bool OrbbecPlaybackCamera::start_camera() {
    assert(camera_stopped);
    assert(!camera_started);
    assert(camera_processing_thread == nullptr);
    if (debug) _log_debug("Starting pipeline");
    auto config = std::make_shared<ob::Config>();
    if (!_init_config_for_this_camera(config)) {
        return false;
    }
    try {
        camera_pipeline.start(config);
    } catch(ob::Error& e) {
        _log_error(std::string("pipeline.start error: ") + e.what());
      return false;
    }

    _post_start_this_camera();
    // xxxjack _computePointSize()??
    camera_started = true;
    return true;
}

void OrbbecPlaybackCamera::_post_start_this_camera() {
    // XXX Implement me
}


bool OrbbecPlaybackCamera::_init_config_for_this_camera(std::shared_ptr<ob::Config> config) {
    _log_error("Not yet implemented");
    return false;
#ifdef xxxjack
    std::shared_ptr<ob::Device> device = camera_pipeline.getDevice();
    // Ensure device clock is synchronized with host clock.
    device->timerSyncWithHost();
    // Ensure frames are synchronized
    camera_pipeline.enableFrameSync();
    // Set a flag if we want to record.
    uses_recorder = record_to_file != "";
    try {
        config->enableVideoStream(OB_STREAM_COLOR, hardware.color_width, hardware.color_height, hardware.fps, OB_FORMAT_BGRA);
        config->enableVideoStream(OB_STREAM_DEPTH, hardware.depth_width, hardware.depth_height, hardware.fps, OB_FORMAT_Y16);
        config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
        // xxxjack this is really only for depth2color mode.
        config->setAlignMode(ALIGN_D2C_HW_MODE);
    } catch(ob::Error& e) {
        _log_error(std::string("enableVideoStream error: ") + e.what());
        return false;
    }
    return _start_recorder();
#endif
}

void OrbbecPlaybackCamera::start_camera_streaming() {
    assert(camera_stopped);
    camera_stopped = false;
    _start_capture_thread();
    _start_processing_thread();
}
