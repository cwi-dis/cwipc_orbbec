#include "OrbbecCapture.hpp"

OrbbecCapture::OrbbecCapture()
: OrbbecBaseCapture<ob::Device, OrbbecCamera>("cwipc_orbbec::OrbbecCapture", "orbbec")
{
}

int OrbbecCapture::countDevices() {
  ob::Context context;
  auto deviceList = context.queryDeviceList();

  return deviceList->deviceCount();
}

bool OrbbecCapture::_apply_default_config() {
  bool hasFailed = false;

  ob::Context context;
  auto deviceList = context.queryDeviceList();

  for (int i = 0; i < deviceList->deviceCount(); i++) {
    OrbbecCameraConfig config;

    config.type = "orbbec";
    config.serial = deviceList->serialNumber(i);

    pcl::shared_ptr<Eigen::Affine3d> default_trafo(new Eigen::Affine3d());
    default_trafo->setIdentity();

    config.trafo = default_trafo;
    config.cameraposition = { 0, 0, 0 };

    configuration.all_camera_configs.push_back(config);
  }

  if (hasFailed) {
    _unload_cameras();
    return false;
  }

  return true;
}

bool OrbbecCapture::openCameras() {
  ob::Context context;
  auto deviceList = context.queryDeviceList();

  for (int i = 0; i < deviceList->deviceCount(); i++) {
    const char* serialNum = deviceList->serialNumber(i);
    OrbbecCameraConfig* config = get_camera_config(serialNum);

    if (config == 0) {
      _log_error(std::string("Camera ") + serialNum + " not found in configuration");
      return false;
    }

    if (config->disabled) {
      config->handle = 0;
    } else {
      config->handle = deviceList->getDevice(i);
    }
  }

  return true;
}

bool OrbbecCapture::initializeHardwareSettings() {
  for (auto &config : configuration.all_camera_configs) {
    if (config.disabled || config.handle == 0) {
      continue;
    }

    ob::Device* device = config.handle.get();

    if (configuration.camera_processing.color_exposure_time >= 0) {
      device->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, configuration.camera_processing.color_exposure_time);
    } else {
      device->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, true);
    }

    if (configuration.camera_processing.color_whitebalance >= 0) {
      device->setIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT, configuration.camera_processing.color_whitebalance);
    } else {
      device->setBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, true);
    }

    if (configuration.camera_processing.color_backlight_compensation >= 0 && device->isPropertySupported(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT, OB_PERMISSION_WRITE)) {
      device->setIntProperty(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT, configuration.camera_processing.color_backlight_compensation);
    }

    if (configuration.camera_processing.color_brightness >= 0) {
      device->setIntProperty(OB_PROP_COLOR_BRIGHTNESS_INT, configuration.camera_processing.color_brightness);
    }

    if (configuration.camera_processing.color_contrast >= 0) {
      device->setIntProperty(OB_PROP_COLOR_CONTRAST_INT, configuration.camera_processing.color_contrast);
    }

    if (configuration.camera_processing.color_saturation >= 0) {
      device->setIntProperty(OB_PROP_COLOR_SATURATION_INT, configuration.camera_processing.color_saturation);
    }

    if (configuration.camera_processing.color_sharpness >= 0) {
      device->setIntProperty(OB_PROP_COLOR_SHARPNESS_INT, configuration.camera_processing.color_sharpness);
    }

    if (configuration.camera_processing.color_gain >= 0) {
      device->setIntProperty(OB_PROP_COLOR_GAIN_INT, configuration.camera_processing.color_gain);
    }

    if (configuration.camera_processing.color_powerline_frequency >= 0) {
      device->setIntProperty(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT, configuration.camera_processing.color_powerline_frequency);
    }
  }

  return true;
}

bool OrbbecCapture::createCameras() {
  for (auto &config : configuration.all_camera_configs) {
    if (config.disabled || config.handle == 0) {
      continue;
    }

    ob::Device* device = config.handle.get();

    if (config.type != type) {
      _log_error(std::string("Camera ") + config.serial + " is not a " + type + " camera");
      return false;
    }

    int cameraIndex = cameras.size();
    auto camera = new Type_our_camera(device, configuration, cameraIndex, config);
    cameras.push_back(camera);
  }

  return true;
}

bool OrbbecCapture::config_reload(const char* configFilename) {
  _unload_cameras();

  if (countDevices() == 0) {
    return false;
  }

  if (!apply_config(configFilename)) {
    camera_count = 0;
    return false;
  }

  if (!openCameras()) {
    camera_count = 0;
    return false;
  }

  if (!initializeHardwareSettings()) {
    camera_count = 0;
    return false;
  }

  if (!createCameras()) {
    _unload_cameras();
    return false;
  }

  _init_camera_positions();
  _start_cameras();

  stopped = false;
  control_thread = new std::thread(&OrbbecCapture::_control_thread_main, this);
  _cwipc_setThreadName(control_thread, L"cwipc_orbbec::control_thread");

  return false;
}

bool OrbbecCapture::_capture_all_cameras() {
  bool capturesOk = true;

  for (auto cam : cameras) {
    if (!cam->capture_frameset()) {
      capturesOk = false;
    }
  }

  return capturesOk;
}

uint64_t OrbbecCapture::_get_best_timestamp() {
  uint64_t timestamp = 0;

  for (auto cam : cameras) {
    uint64_t cameraTimestamp = cam->get_capture_timestamp();

    if (cameraTimestamp > timestamp) {
      timestamp = cameraTimestamp;
    }
  }

  if (timestamp <= 0) {
    timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  }

  return timestamp;
}

bool OrbbecCapture::seek(uint64_t timestamp) {
  return false;
}
