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
  // XXX IMPLEMENT ME
  assert(0);
  return false;
}

void OrbbecCamera::stop_camera() {
  // XXX IMPLEMENT ME
  assert(0);
}

bool OrbbecCamera::_init_hardware_for_this_camera() {

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

void OrbbecCamera::start_camera_streaming() {
  // XXX IMPLEMENT ME
  camera_pipeline.start();
  _start_capture_thread();
}

uint64_t OrbbecCamera::wait_for_captured_frameset(uint64_t minimum_timestamp) {
  // XXX IMPLEMENT ME
  current_captured_frameset = camera_pipeline.waitForFrameset();
  return true;
}

void OrbbecCamera::_start_capture_thread() {
  // XXX IMPLEMENT ME
  assert(!camera_started);
  assert(camera_stopped);
  camera_capturer_thread = new std::thread(&OrbbecCamera::_capture_thread_main, this);
  // xxxjack _cwipc_setThreadName(grabber_thread, L"cwipc_orbbec::OrbecCamera::grabber_thread");
}

void OrbbecCamera::_capture_thread_main() {
  // XXX IMPLEMENT ME
  camera_started = true;
}
