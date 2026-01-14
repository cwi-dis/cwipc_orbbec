#pragma once

#include "libobsensor/hpp/Device.hpp"
#include "OrbbecBaseCamera.hpp"

class OrbbecCamera : public OrbbecBaseCamera<std::shared_ptr<ob::Device>> {
    typedef std::shared_ptr<ob::Device> Type_api_camera;

    OrbbecCamera(const OrbbecCamera&);
    OrbbecCamera& operator=(const OrbbecCamera&);

public:
    OrbbecCamera(Type_api_camera camera, OrbbecCaptureConfig& config, int cameraIndex);

    virtual ~OrbbecCamera() {
    }

    // pre_start_all_cameras() defined in base class
    bool start_camera() override;
    virtual void start_camera_streaming() override;
    // pre_stop_camera() defined in base class
    void stop_camera() override;
    virtual uint64_t wait_for_captured_frameset(uint64_t minimum_timestamp) override final;

protected:
    virtual bool _init_hardware_for_this_camera() override final;
    virtual void _start_capture_thread() override;
    virtual void _capture_thread_main() override;
};
