#pragma once

#include <string>

#include "libobsensor/hpp/Frame.hpp"
#include "libobsensor/hpp/Pipeline.hpp"
#include "readerwriterqueue.h"

#include "cwipc_util/capturers.hpp"
#include "OrbbecConfig.hpp"
typedef void *xxxjack_dont_know_yet;
template<typename Type_api_camera> class OrbbecBaseCamera : public CwipcBaseCamera {
protected:
  std::thread *capture_thread;
  std::string CLASSNAME;
  OrbbecCaptureConfig& configuration;
  Type_api_camera* camera_handle;
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

public:
  std::string serial;
  bool eof = false;
  int camera_index;

  OrbbecBaseCamera(const std::string& _Classname, Type_api_camera* _handle, OrbbecCaptureConfig& _configuration, int _camera_index, OrbbecCameraConfig& _camera_configuration) :
    CLASSNAME(_Classname),
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
    processing_frame_queue(1) {
  }

  virtual ~OrbbecBaseCamera() {
  }

  virtual bool start() = 0;
  virtual void start_capturer() = 0;
  virtual void stop() = 0;

  virtual bool isSyncMaster() {
    return camera_sync_is_master;
  }

  virtual void create_pc_from_frames() final {
    assert(current_frameset && current_frameset.getImpl());

    if (!processing_frame_queue.try_enqueue(current_frameset)) {
        std::cerr << CLASSNAME << ":  camera " << serial << ": drop frame before processing" << std::endl;
        // xxxjack it seems Orbbec does this automatically. k4a_capture_release(current_frameset);
    }

}

virtual void wait_for_pc() final {
    std::unique_lock<std::mutex> lock(processing_mutex);
    processing_done_cv.wait(lock, [this] { return processing_done; });
    processing_done = false;
}

  virtual uint64_t get_capture_timestamp() final {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  }

  virtual cwipc_pcl_pointcloud get_current_pointcloud() final {
      return current_pointcloud;
  }
};
