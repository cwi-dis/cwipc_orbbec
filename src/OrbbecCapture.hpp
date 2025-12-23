#pragma once

#include "libobsensor/hpp/Context.hpp"

#include "OrbbecBaseCapture.hpp"
#include "OrbbecCamera.hpp"

class OrbbecCapture : public OrbbecBaseCapture<ob::Device, OrbbecCamera> {
  typedef ob::Device Type_api_camera;
  typedef OrbbecCamera Type_our_camera;

  virtual bool _apply_default_config() override;
  bool initializeHardwareSettings();
  bool createCameras();
  bool openCameras();

protected:
  OrbbecCapture();

  bool _capture_all_cameras() override;
  uint64_t _get_best_timestamp() override;

public:
  static int countDevices();

  static OrbbecCapture* factory() {
    return new OrbbecCapture();
  }

  virtual ~OrbbecCapture() {
  }

  virtual bool config_reload(const char* configFilename) override;
  bool seek(uint64_t timestamp) override;
};
