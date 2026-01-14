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
  return true;
}

bool OrbbecCapture::_create_cameras() {
  ob::Context context;
  auto deviceList = context.queryDeviceList();

  for (int i = 0; i < deviceList->deviceCount(); i++) {
    Type_api_camera handle; // xxxjack to be done
    const char* serialNum = deviceList->serialNumber(i);
    OrbbecCameraConfig* cd = get_camera_config(serialNum);

    if (cd == nullptr) {
      _log_error(std::string("Camera with serial ") + serialNum + " is connected but not in configuration");
      return false;
    }

    if (cd->type != "orbbec") {
      _log_error(std::string("Camera ") + serialNum + " is type " + cd->type + " in stead of orbbec");
      return false;
    }
    if (cd->disabled) {
      // xxxjack do we need to close it?
    } else {
      int camera_index = cameras.size();
      auto cam = _create_single_camera(handle, configuration, camera_index);
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
    uint64_t first_timestamp = 0;
    for(auto cam : cameras) {
        uint64_t this_cam_timestamp = cam->wait_for_captured_frameset(first_timestamp);
        if (this_cam_timestamp == 0) {
            _log_warning("no frameset captured from camera");
            return false;
        }
        if (first_timestamp == 0) {
            first_timestamp = this_cam_timestamp;
        }
    }

    // And get the best timestamp
    if (configuration.new_timestamps) {
        timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    } else {
        timestamp = first_timestamp;
    }
    return true;
}

bool OrbbecCapture::seek(uint64_t timestamp) {
  return false;
}
