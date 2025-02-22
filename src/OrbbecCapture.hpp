#pragma once

#include "libobsensor/hpp/Context.hpp"

#include "OrbbecBaseCapture.hpp"
#include "OrbbecCamera.hpp"

class OrbbecCapture : public OrbbecBaseCapture<OrbbecCamera> {
  typedef ob::Device ApiCameraType;
  typedef OrbbecCamera CameraType;

private:
  virtual bool applyDefaultConfig() override;
  bool initializeHardwareSettings();
  bool createCameras();
  bool openCameras();

protected:
  OrbbecCapture();

public:
  static int countDevices();

  static OrbbecCapture* factory() {
    return new OrbbecCapture();
  }

  virtual ~OrbbecCapture() {
  }

  virtual bool configReload(const char* configFilename) override;
  bool seek(uint64_t timestamp) override;
};
