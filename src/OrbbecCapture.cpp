#include "OrbbecCapture.hpp"

OrbbecCapture::OrbbecCapture()
: OrbbecBaseCapture<Type_api_camera,Type_our_camera>("cwipc_orbbec::OrbbecCapture", "orbbec")
{
}

int OrbbecCapture::countDevices() {
  ob::Context context;
  auto deviceList = context.queryDeviceList();

  return deviceList->deviceCount();
}

bool OrbbecCapture::_apply_auto_config() {
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


bool OrbbecCapture::_init_hardware_for_all_cameras() {
  // xxxjack needs to go to per-camera
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

bool OrbbecCapture::_create_cameras() {
  ob::Context context;
  auto deviceList = context.queryDeviceList();

  for (int i = 0; i < deviceList->deviceCount(); i++) {
    Type_api_camera handle;
    const char* serialNum = deviceList->serialNumber(i);
    OrbbecCameraConfig* cd = get_camera_config(serialNum);

    if (cd == nullptr) {
      _log_error(std::string("Camera with serial ") + serialNum + " is connected but not in configuration");
      return false;
    }

    if (cd->type != "orbbect") {
      _log_error(std::string("Camera ") + serialNum + " is type " + cd->type + " in stead of orbbec");
      return false;
    }
    if (cd->disabled) {
      // xxxjack do we need to close it?
    } else {
      int camera_index = cameras.size();
      auto cam = _create_single_camera(handle, configuration, camera_index, *cd);
      cameras.push_back(cam);
      cd->connected = true;
    }
  }
  return true;
}


bool OrbbecCapture::_check_cameras_connected() {
    for (auto& cd : configuration.all_camera_configs) {
        if (!cd.connected && !cd.disabled) {
            _log_warning("Camera with serial " + cd.serial + " is not connected");
            return false;
        }
    }
    return true;
}

bool OrbbecCapture::_capture_all_cameras(uint64_t& timestamp) {
  bool capturesOk = true;
  timestamp = 0;
  for (auto cam : cameras) {
    if (!cam->capture_frameset()) {
      capturesOk = false;
      continue;
    }
    uint64_t camts = cam->get_capture_timestamp();
    if (camts > timestamp) {
        timestamp = camts;
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
