#include "OrbbecCamera.hpp"
#include "OrbbecConfig.hpp"

OrbbecCamera::OrbbecCamera(ApiCameraType* camera, OrbbecCaptureConfig& config, int cameraIndex, OrbbecCameraConfig& cameraConfig) : OrbbecBaseCamera("cwipc_orbbec: OrbbecCamera", camera, config, cameraIndex, cameraConfig) {
  if (config.record_to_directory != "") {
    recordToFile = config.record_to_directory + "/" + cameraConfig.serial + ".mkv";
  }
}

bool OrbbecCamera::start() {
  // XXX IMPLEMENT ME
  return false;
}

void OrbbecCamera::stop() {
  // XXX IMPLEMENT ME
}

void OrbbecCamera::startCapturer() {
  // XXX IMPLEMENT ME
}

bool OrbbecCamera::captureFrameset() {
  // XXX IMPLEMENT ME
  return false;
}

void OrbbecCamera::startCaptureThread() {
  // XXX IMPLEMENT ME
}

void OrbbecCamera::captureThreadFunction() {
  // XXX IMPLEMENT ME
}
