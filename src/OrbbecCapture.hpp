#pragma once

#include "libobsensor/hpp/Context.hpp"

#include "OrbbecBaseCapture.hpp"
#include "OrbbecCamera.hpp"

class OrbbecCapture : public OrbbecBaseCapture<std::shared_ptr<ob::Device>, OrbbecCamera> {
  typedef std::shared_ptr<ob::Device> Type_api_camera;
  typedef OrbbecCamera Type_our_camera;

  virtual bool _apply_auto_config() override final;
  virtual bool _init_hardware_for_all_cameras() override final;
  virtual bool _create_cameras() override final;
  virtual bool _check_cameras_connected() override final;

protected:
  OrbbecCapture();

public:
  static int countDevices();

  static OrbbecCapture* factory() {
    return new OrbbecCapture();
  }

  virtual ~OrbbecCapture() {
  }

  bool seek(uint64_t timestamp) override;
};
