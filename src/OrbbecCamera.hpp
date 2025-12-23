#pragma once

#include "libobsensor/hpp/Device.hpp"
#include "OrbbecBaseCamera.hpp"

class OrbbecCamera : public OrbbecBaseCamera<ob::Device> {
  typedef ob::Device Type_api_camera;

  OrbbecCamera(const OrbbecCamera&);
  OrbbecCamera& operator=(const OrbbecCamera&);

protected:
  virtual void _start_capture_thread() override;
  virtual void _capture_thread_main() override;

public:
  OrbbecCamera(Type_api_camera* camera, OrbbecCaptureConfig& config, int cameraIndex, OrbbecCameraConfig& cameraConfig);

  virtual ~OrbbecCamera() {
  }

  bool start() override;
  void stop() override;
  virtual void start_capturer() override;
  bool capture_frameset();
};
