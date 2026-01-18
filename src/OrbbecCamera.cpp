#include "OrbbecCamera.hpp"
#include "OrbbecConfig.hpp"

OrbbecCamera::OrbbecCamera(std::shared_ptr<ob::Device> camera, OrbbecCaptureConfig& config, int cameraIndex)
: OrbbecBaseCamera("cwipc_orbbec: OrbbecCamera", camera, config, cameraIndex)
{
    if (config.record_to_directory != "") {
        record_to_file = config.record_to_directory + "/" + camera_config.serial + ".mkv";
    }
}

bool OrbbecCamera::start_camera() {
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

void OrbbecCamera::_post_start_this_camera() {
    // XXX Implement me
}

void OrbbecCamera::stop_camera() {
    if (debug) _log_debug("stop camera");
    camera_stopped = true;
    // xxxjack realsense clears out processing_frame_queue...
    if (camera_processing_thread) {
        camera_processing_thread->join();
    }
    delete camera_processing_thread;
    camera_processing_thread = nullptr;

    if (camera_started) {
        camera_pipeline.stop();
    }
    processing_done = true;
    processing_done_cv.notify_one();
    if (debug) _log_debug("camera stopped");
}

bool OrbbecCamera::_init_hardware_for_this_camera() {
    auto camera_handle = camera_pipeline.getDevice();
    if (hardware.color_exposure_time >= 0) {
        camera_handle->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, hardware.color_exposure_time);
    } else {
        camera_handle->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, true);
    }

    if (hardware.color_whitebalance >= 0) {
        camera_handle->setIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT, hardware.color_whitebalance);
    } else {
        camera_handle->setBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, true);
    }

    if (hardware.color_backlight_compensation >= 0 && camera_handle->isPropertySupported(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT, OB_PERMISSION_WRITE)) {
        camera_handle->setIntProperty(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT, hardware.color_backlight_compensation);
    }

    if (hardware.color_brightness >= 0) {
        camera_handle->setIntProperty(OB_PROP_COLOR_BRIGHTNESS_INT, hardware.color_brightness);
    }

    if (hardware.color_contrast >= 0) {
        camera_handle->setIntProperty(OB_PROP_COLOR_CONTRAST_INT, hardware.color_contrast);
    }

    if (hardware.color_saturation >= 0) {
        camera_handle->setIntProperty(OB_PROP_COLOR_SATURATION_INT, hardware.color_saturation);
    }

    if (hardware.color_sharpness >= 0) {
        camera_handle->setIntProperty(OB_PROP_COLOR_SHARPNESS_INT, hardware.color_sharpness);
    }

    if (hardware.color_gain >= 0) {
        camera_handle->setIntProperty(OB_PROP_COLOR_GAIN_INT, hardware.color_gain);
    }

    if (hardware.color_powerline_frequency >= 0) {
        camera_handle->setIntProperty(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT, hardware.color_powerline_frequency);
    }
    return true;
}

bool OrbbecCamera::_init_config_for_this_camera(std::shared_ptr<ob::Config> config) {
    try {
        config->enableVideoStream(OB_STREAM_COLOR, hardware.color_width, hardware.color_height, hardware.fps, OB_FORMAT_BGRA);
        config->enableVideoStream(OB_STREAM_DEPTH, hardware.depth_width, hardware.depth_height, hardware.fps, OB_FORMAT_Y16);
        config->setAlignMode(ALIGN_D2C_HW_MODE);
    } catch(ob::Error& e) {
        _log_error(std::string("enableVideoStream error: ") + e.what());
        return false;
    }
    return true;
}

void OrbbecCamera::start_camera_streaming() {
    assert(camera_stopped);
    camera_stopped = false;
    _start_capture_thread();
    _start_processing_thread();
}

void OrbbecCamera::_start_capture_thread() {
}

void OrbbecCamera::_capture_thread_main() {
}
