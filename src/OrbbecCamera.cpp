#include "OrbbecCamera.hpp"
#include "OrbbecConfig.hpp"

OrbbecCamera::OrbbecCamera(Type_api_camera* camera, OrbbecCaptureConfig& config, int cameraIndex, OrbbecCameraConfig& cameraConfig) : OrbbecBaseCamera("cwipc_orbbec: OrbbecCamera", camera, config, cameraIndex, cameraConfig) {
  if (config.record_to_directory != "") {
    record_to_file = config.record_to_directory + "/" + cameraConfig.serial + ".mkv";
  }
}

bool OrbbecCamera::start() {
  // XXX IMPLEMENT ME
  assert(0);
  return false;
}

void OrbbecCamera::stop() {
  // XXX IMPLEMENT ME
  assert(0);
}

void OrbbecCamera::start_capturer() {
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
