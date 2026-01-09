#include "OrbbecCamera.hpp"
#include "OrbbecConfig.hpp"

OrbbecCamera::OrbbecCamera(std::shared_ptr<ob::Device> camera, OrbbecCaptureConfig& config, int cameraIndex, OrbbecCameraConfig& cameraConfig)
: OrbbecBaseCamera("cwipc_orbbec: OrbbecCamera", camera, config, cameraIndex, cameraConfig)
{
  if (config.record_to_directory != "") {
    record_to_file = config.record_to_directory + "/" + cameraConfig.serial + ".mkv";
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

    if (configuration.camera_processing.color_exposure_time >= 0) {
      camera_handle->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, configuration.camera_processing.color_exposure_time);
    } else {
      camera_handle->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, true);
    }

    if (configuration.camera_processing.color_whitebalance >= 0) {
      camera_handle->setIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT, configuration.camera_processing.color_whitebalance);
    } else {
      camera_handle->setBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, true);
    }

    if (configuration.camera_processing.color_backlight_compensation >= 0 && camera_handle->isPropertySupported(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT, OB_PERMISSION_WRITE)) {
      camera_handle->setIntProperty(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT, configuration.camera_processing.color_backlight_compensation);
    }

    if (configuration.camera_processing.color_brightness >= 0) {
      camera_handle->setIntProperty(OB_PROP_COLOR_BRIGHTNESS_INT, configuration.camera_processing.color_brightness);
    }

    if (configuration.camera_processing.color_contrast >= 0) {
      camera_handle->setIntProperty(OB_PROP_COLOR_CONTRAST_INT, configuration.camera_processing.color_contrast);
    }

    if (configuration.camera_processing.color_saturation >= 0) {
      camera_handle->setIntProperty(OB_PROP_COLOR_SATURATION_INT, configuration.camera_processing.color_saturation);
    }

    if (configuration.camera_processing.color_sharpness >= 0) {
      camera_handle->setIntProperty(OB_PROP_COLOR_SHARPNESS_INT, configuration.camera_processing.color_sharpness);
    }

    if (configuration.camera_processing.color_gain >= 0) {
      camera_handle->setIntProperty(OB_PROP_COLOR_GAIN_INT, configuration.camera_processing.color_gain);
    }

    if (configuration.camera_processing.color_powerline_frequency >= 0) {
      camera_handle->setIntProperty(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT, configuration.camera_processing.color_powerline_frequency);
    }
    return true;
}

void OrbbecCamera::start_camera_streaming() {
  // XXX IMPLEMENT ME
  camera_pipeline.start();
  _start_capture_thread();
}

bool OrbbecCamera::capture_frameset() {
  // XXX IMPLEMENT ME
  current_frameset = camera_pipeline.waitForFrameset();
  return true;
}

void OrbbecCamera::_start_capture_thread() {
  // XXX IMPLEMENT ME
  assert(!camera_started);
  assert(camera_stopped);
  grabber_thread = new std::thread(&OrbbecCamera::_capture_thread_main, this);
  // xxxjack _cwipc_setThreadName(grabber_thread, L"cwipc_orbbec::OrbecCamera::grabber_thread");
}

void OrbbecCamera::_capture_thread_main() {
  // XXX IMPLEMENT ME
  camera_started = true;
  while(!camera_stopped) {
    bool ok = capture_frameset();
  }
}
