#pragma once

#include <string>

#include "libobsensor/hpp/Frame.hpp"
#include "libobsensor/hpp/Pipeline.hpp"
#include "readerwriterqueue.h"

#include "cwipc_util/capturers.hpp"
#include "OrbbecConfig.hpp"
typedef void *xxxjack_dont_know_yet;
template<typename Type_api_camera> 
class OrbbecBaseCamera : public CwipcBaseCamera {


public:
    OrbbecBaseCamera(const std::string& _Classname, Type_api_camera _handle, OrbbecCaptureConfig& _configuration, int _camera_index, OrbbecCameraConfig& _camera_configuration)
    : CwipcBaseCamera(_Classname + ": " + _camera_configuration.serial, "orbbec"),
      camera_handle(_handle),
      configuration(_configuration),
      camera_index(_camera_index),
      camera_configuration(_camera_configuration),
      serial(_camera_configuration.serial),
      current_frameset(nullptr),
      camera_sync_is_master(_camera_configuration.serial == _configuration.sync_master_serial),
      camera_sync_is_used(_configuration.sync_master_serial != ""),
      do_height_filtering(_configuration.height_min != _configuration.height_max),
      captured_frame_queue(1),
      processing_frame_queue(1)
  {
  }

    virtual ~OrbbecBaseCamera() {
      // xxxjack implement proper stopping and cleanup
    }

    /// Step 1 in starting: tell the camera we are going to start. Called for all cameras.
    virtual bool pre_start_all_cameras() final { 
      if (!_init_filters()) {
          return false;
      }
      if (!_init_hardware_for_this_camera()) {
          return false;
      }
      return true;
    }
    /// Step 2 in starting: starts the camera. Called for all cameras. 
    virtual bool start_camera() override = 0;
    /// Step 3 in starting: starts the capturer. Called after all cameras have been started.
    virtual void start_camera_streaming() override = 0;
    /// Step 4, called after all capturers have been started.
    virtual void post_start_all_cameras() override final {

    }
    virtual void pre_stop_camera() override final {
      
    }
    virtual void stop_camera() override = 0;


    virtual bool is_sync_master() override final {
      return camera_sync_is_master;
    }
protected:
    // internal API that is "shared" with other implementations (realsense, kinect)
    /// Initialize any hardware settings for this camera.
    /// Also see xxxjack
    virtual bool _init_hardware_for_this_camera() override = 0;
    /// Initialize any filters that will be applied to all RGB/D images.
    virtual bool _init_filters() override final {
      // Orbbec API does not provide any filtering that needs to be initialized.
      return true;
    }
    /// Apply filter to a frameset.
    virtual void _apply_filters() final {
      // xxxjack to be implemented, maybe
    }
    /// Initialize the body tracker
    virtual bool _init_tracker() override final { 
      // Body tracker not supported for Orbbec
      return true; 
    }
    /// Create per-API configuration for starting the camera 
    /// virtual void _prepare_config_for_starting_camera(...) = 0;
public:
  virtual void process_pointcloud_from_frameset() final {
    assert(current_frameset && current_frameset.getImpl());

    if (!processing_frame_queue.try_enqueue(current_frameset)) {
        _log_warning("processing_frame_queue full, dropping frame");
        // xxxjack it seems Orbbec does this automatically. k4a_capture_release(current_frameset);
    }
    current_frameset = nullptr;
  }

virtual void wait_for_pointcloud_processed() final {
    std::unique_lock<std::mutex> lock(processing_mutex);
    processing_done_cv.wait(lock, [this] { return processing_done; });
    processing_done = false;
}

  virtual uint64_t get_capture_timestamp() final {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  }

  virtual cwipc_pcl_pointcloud access_current_pcl_pointcloud() final {
      return current_pointcloud;
  }

  void save_frameset_auxdata(cwipc *pc) {
    // xxxjack to be implemented
  }
  bool map2d3d(int x_2d, int y_2d, int d_2d, float* out3d) {
    // xxxjack to be implemented
    return false;
  }
public:
  float pointSize = 0;
  std::string serial;
  bool _eof = false;
  int camera_index;

protected:
  std::thread *grabber_thread;
  OrbbecCaptureConfig& configuration;
  Type_api_camera camera_handle;
  bool stopped = true;
  bool camera_started = false;
  bool camera_stopped = true;

  OrbbecCameraConfig& camera_configuration;

  bool processing_done = false;

  cwipc_pcl_pointcloud current_pointcloud = nullptr;  //<! Most recent grabbed pointcloud
  ob::Pipeline camera_pipeline;
  moodycamel::BlockingReaderWriterQueue<std::shared_ptr<ob::FrameSet>> captured_frame_queue;
  moodycamel::BlockingReaderWriterQueue<std::shared_ptr<ob::FrameSet>> processing_frame_queue;
  std::shared_ptr<ob::FrameSet> current_frameset;
  bool camera_sync_is_master;
  bool camera_sync_is_used;
  bool do_height_filtering;
  std::mutex processing_mutex;  //<! Exclusive lock for frame to pointcloud processing.
  std::condition_variable processing_done_cv; //<! Condition variable signalling pointcloud ready

  std::string record_to_file;


  virtual void _start_capture_thread() = 0;
  virtual void _capture_thread_main() = 0;
};
