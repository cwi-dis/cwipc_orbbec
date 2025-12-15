#include "OrbbecCamera.hpp"
#include "OrbbecConfig.hpp"

OrbbecCamera::OrbbecCamera(ApiCameraType* camera, OrbbecCaptureConfig& config, int cameraIndex, OrbbecCameraConfig& cameraConfig) : OrbbecBaseCamera("cwipc_orbbec: OrbbecCamera", camera, config, cameraIndex, cameraConfig) {
  if (config.record_to_directory != "") {
    record_to_file = config.record_to_directory + "/" + cameraConfig.serial + ".mkv";
  }
}

bool OrbbecCamera::start() {
  // XXX IMPLEMENT ME
  return false;
}

void OrbbecCamera::stop() {
  // XXX IMPLEMENT ME
}

void OrbbecCamera::start_capturer() {
  // XXX IMPLEMENT ME
}

bool OrbbecCamera::capture_frameset() {
  // XXX IMPLEMENT ME
  return false;
}

void OrbbecCamera::_start_capture_thread() {
  // XXX IMPLEMENT ME
}

void OrbbecCamera::_capture_thread_main() {
  // XXX IMPLEMENT ME
}
