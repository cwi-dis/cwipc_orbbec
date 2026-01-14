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
  auto config = std::make_shared<ob::Config>();
  _init_config_for_this_camera(config);
  camera_pipeline.start();
  _post_start_this_camera();
  // xxxjack _computePointSize()??
  camera_started = true;
  return true;
}

void OrbbecCamera::_post_start_this_camera() {
  // XXX Implement me
}

void OrbbecCamera::stop_camera() {
  camera_pipeline.stop();
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
  config->enableVideoStream(OB_STREAM_COLOR, hardware.color_width, hardware.color_height, OB_FORMAT_BGRA);
  config->enableVideoStream(OB_STREAM_DEPTH, hardware.depth_width, hardware.depth_height, OB_FORMAT_Y16);
  return true;
}

void OrbbecCamera::start_camera_streaming() {
  assert(camera_stopped);
  camera_stopped = false;
  _start_capture_thread();
  _start_processing_thread();
}

uint64_t OrbbecCamera::wait_for_captured_frameset(uint64_t minimum_timestamp) {
  if (camera_stopped) return 0;
  uint64_t resultant_timestamp = 0;
  do {
    current_captured_frameset = camera_pipeline.waitForFrameset();
    std::shared_ptr<ob::Frame> depth_frame = current_captured_frameset->getFrame(OB_FRAME_DEPTH);
    resultant_timestamp = depth_frame->getTimeStampUs();
    if (resultant_timestamp < minimum_timestamp) {
      _log_trace("drop frame with dts=" + std::to_string(resultant_timestamp));
    }
  } while (resultant_timestamp < minimum_timestamp);
  return resultant_timestamp;
}

void OrbbecCamera::_start_capture_thread() {
}

void OrbbecCamera::_capture_thread_main() {
}
