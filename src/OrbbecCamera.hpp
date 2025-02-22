#pragma once

#include "libobsensor/hpp/Device.hpp"
#include "OrbbecBaseCamera.hpp"

class OrbbecCamera : public OrbbecBaseCamera<ob::Device> {
  typedef ob::Device ApiCameraType;

  OrbbecCamera(const OrbbecCamera&);
  OrbbecCamera& operator=(const OrbbecCamera&);

protected:
  virtual void startCaptureThread() override;
  virtual void captureThreadFunction() override;

public:
  OrbbecCamera(ApiCameraType* camera, OrbbecCaptureConfig& config, int cameraIndex, OrbbecCameraConfig& cameraConfig);

  virtual ~OrbbecCamera() {
  }

  bool start() override;
  void stop() override;
  virtual void startCapturer() override;
  bool captureFrameset();
};
