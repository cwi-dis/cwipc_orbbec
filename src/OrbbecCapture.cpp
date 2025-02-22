#include "OrbbecCapture.hpp"

OrbbecCapture::OrbbecCapture() : OrbbecBaseCapture("cwipc_orbbec: OrbbecCapture"){
  type = "orbbec";
}

int OrbbecCapture::countDevices() {
  ob::Context context;
  auto deviceList = context.queryDeviceList();

  return deviceList->deviceCount();
}

bool OrbbecCapture::applyDefaultConfig() {
  bool hasFailed = false;

  ob::Context context;
  auto deviceList = context.queryDeviceList();

  for (int i = 0; i < deviceList->deviceCount(); i++) {
    OrbbecCameraConfig config;

    config.serial = deviceList->serialNumber(i);

    pcl::shared_ptr<Eigen::Affine3d> trafo(new Eigen::Affine3d());
    trafo->setIdentity();

    config.trafo = trafo;
    config.intrinsicTrafo = 0;
    config.cameraposition = { 0, 0, 0 };

    configuration.all_camera_configs.push_back(config);
  }

  if (hasFailed) {
    unloadCameras();
    return false;
  }

  return true;
}

bool OrbbecCapture::openCameras() {
  ob::Context context;
  auto deviceList = context.queryDeviceList();

  for (int i = 0; i < deviceList->deviceCount(); i++) {
    const char* serialNum = deviceList->serialNumber(i);
    OrbbecCameraConfig* config = getCameraConfig(serialNum);

    if (config == 0) {
      cwipc_orbbec_log_warning(std::string("Camera ") + serialNum + " not found in configuration");
      continue;
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

    if (configuration.camera_processing.color_backlight_compensation >= 0) {
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
      cwipc_orbbec_log_warning(std::string("Camera ") + config.serial + " is not a " + type + " camera");
      continue;
    }

    int cameraIndex = cameras.size();
    auto camera = new CameraType(device, configuration, cameraIndex, config);
    cameras.push_back(camera);
  }

  return true;
}

bool OrbbecCapture::configReload(const char* configFilename) {
  unloadCameras();

  if (countDevices() == 0) {
    return false;
  }

  if (!applyConfig(configFilename)) {
    cameraCount = 0;
    return false;
  }

  if (!openCameras()) {
    cameraCount = 0;
    return false;
  }

  if (!initializeHardwareSettings()) {
    cameraCount = 0;
    return false;
  }

  if (!createCameras()) {
    unloadCameras();
    return false;
  }

  initCameraPositions();
  startCameras();

  stopped = false;
  controlThread = new std::thread(&OrbbecCapture::controlThreadFunction, this);
  cwipc_setThreadName(controlThread, L"cwipc_orbbec::controlThread");

  return false;
}

bool OrbbecCapture::seek(uint64_t timestamp) {
  return false;
}
