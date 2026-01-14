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
    OrbbecBaseCamera(const std::string& _Classname, Type_api_camera _handle, OrbbecCaptureConfig& _configuration, int _camera_index)
    : CwipcBaseCamera(_Classname + ": " + _configuration.all_camera_configs[_camera_index].serial, "orbbec"),
      camera_index(_camera_index),
      configuration(_configuration),
      serial(_configuration.all_camera_configs[_camera_index].serial),
      camera_sync_ismaster(serial == configuration.sync.sync_master_serial),
      camera_stopped(true),
      camera_config(_configuration.all_camera_configs[_camera_index]),
      filtering(_configuration.filtering),
      processing(_configuration.processing),
      hardware(_configuration.hardware),
      auxData(_configuration.auxData),
      camera_handle(_handle),
      captured_frame_queue(1),
      processing_frame_queue(1),
      camera_sync_inuse(configuration.sync.sync_master_serial != ""),
      current_captured_frameset(nullptr)
      
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
      return camera_sync_ismaster;
    }

    virtual bool mapcolordepth(int x_c, int y_c, int* out2d) override final {
      // xxxjack to be implemented
      return false;
    }
    virtual bool map2d3d(int x_2d, int y_2d, int d_2d, float* out3d) override final {
      // xxxjack to be implemented
      return false;
    }
    /// Get current camera hardware parameters.
    /// xxxjack to be overridden for real cameras.
    virtual void get_camera_hardware_parameters(OrbbecCameraHardwareConfig& output) {
        output = hardware;
    }
    /// Return true if current hardware parameters of this camera match input.
    bool match_camera_hardware_parameters(OrbbecCameraHardwareConfig& input) {
        return (
            false
        );
    }

public:
    /// Step 1 in capturing: wait for a valid frameset. Any image processing will have been done. 
    /// Returns timestamp of depth frame, or zero if none available.
    /// This ensures protected attribute current_captured_frameset is valid.
    virtual uint64_t wait_for_captured_frameset(uint64_t minimum_timestamp) = 0;
    /// Step 2: Forward the current_captured_frameset to the processing thread to turn it into a point cloud.
    virtual void process_pointcloud_from_frameset() final {
      assert(current_captured_frameset && current_captured_frameset.getImpl());

      if (!processing_frame_queue.try_enqueue(current_captured_frameset)) {
          _log_warning("processing_frame_queue full, dropping frame");
          // xxxjack it seems Orbbec does this automatically. k4a_capture_release(current_captured_frameset);
      }
      current_captured_frameset = nullptr;
    }
    /// Step 3: Wait for the point cloud processing.
    /// After this, current_pcl_pointcloud and current_processed_frameset will be valid.
    virtual void wait_for_pointcloud_processed() final {
        std::unique_lock<std::mutex> lock(processing_mutex);
        processing_done_cv.wait(lock, [this] { return processing_done; });
        processing_done = false;
    }
    /// Step 4: borrow a pointer to the point cloud just created, as a PCL point cloud.
    /// The capturer will use this to populate the resultant cwipc point cloud with points
    /// from all cameras.
    cwipc_pcl_pointcloud access_current_pcl_pointcloud() { return current_pcl_pointcloud; }
    /// Step 5: Save auxdata from frameset into given cwipc object.
    void save_frameset_auxdata(cwipc *pc) {
      // xxxjack to be implemented
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
    virtual bool _init_skeleton_tracker() override final { 
      // Body tracker not supported for Orbbec
      return true; 
    }

    virtual void _apply_filters() final {
      // xxxjack to be implemented, maybe
    }
    virtual void _start_capture_thread() = 0;
    virtual void _capture_thread_main() = 0;
    void _start_processing_thread() {
        camera_processing_thread = new std::thread(&OrbbecBaseCamera::_processing_thread_main, this);
        _cwipc_setThreadName(camera_processing_thread, L"cwipc_orbbec::camera_processing_thread");
    }

    void _processing_thread_main() {
#if 0
        _log_debug_thread("frame processing thread started for camera " + serial);
        while (!camera_stopped) {
            //
            // Get the frameset we need to turn into a point cloud
            ///
            k4a_capture_t processing_frameset = NULL;
            bool ok = processing_frame_queue.wait_dequeue_timed(processing_frameset, std::chrono::milliseconds(10000));

            if (!ok) {
                _log_warning("processing thread dequeue timeout");
                continue;
            }

            _log_debug_thread("processing thread got frameset for camera " + serial);
            assert(processing_frameset);
            
            std::lock_guard<std::mutex> lock(processing_mutex);
            //
            // use body tracker for skeleton extraction
            //
            if (tracker_handle) {
                _feed_frameset_to_tracker(processing_frameset);
            }
            //
            // get depth and color images. Apply filters and uncompress color image if needed
            //
            k4a_image_t depth_image = k4a_capture_get_depth_image(processing_frameset);
            k4a_image_t color_image = k4a_capture_get_color_image(processing_frameset);
            //
            // Do processing on the images (filtering, decompressing)
            //
            _apply_filters_to_depth_image(depth_image); //filtering depthmap => better now because if we map depth to color then we need to filter more points.
            color_image = _uncompress_color_image(processing_frameset, color_image);
            //  
            // generate pointcloud
            //
            cwipc_pcl_pointcloud new_pointcloud = nullptr;
            if (filtering.map_color_to_depth) {
                new_pointcloud = _generate_point_cloud_color_to_depth(depth_image, color_image);
            } else {
                new_pointcloud = _generate_point_cloud_depth_to_color(depth_image, color_image);
            }

            if (new_pointcloud != nullptr) {
                current_pcl_pointcloud = new_pointcloud;
                _log_debug_thread("generated pointcloud with " + std::to_string(current_pcl_pointcloud->size()) + " points");

                if (current_pcl_pointcloud->size() == 0) {
                    _log_warning("Captured empty pointcloud from camera");
                    //continue;
                }
                //
                // Notify wait_for_pointcloud_processed that we're done.
                //
                processing_done = true;
                processing_done_cv.notify_one();
            }
            //
            // Release resources no longer needed.
            //
            if (depth_image) {
                k4a_image_release(depth_image);
            }
            if (color_image) {
                k4a_image_release(color_image);
            }
            if (processing_frameset) {
                k4a_capture_release(processing_frameset);
            }
        }
#endif
        _log_debug_thread("processing thread exiting");
    }
public:
  float pointSize = 0;
  std::string serial;
  bool end_of_stream_reached = false;
  int camera_index;

protected:
  OrbbecCaptureConfig configuration;
  OrbbecCameraConfig& camera_config;
  OrbbecCaptureProcessingConfig& processing;
  OrbbecCameraProcessingParameters& filtering;
  OrbbecCameraHardwareConfig& hardware;
  OrbbecCaptureAuxDataConfig& auxData; //<! Auxiliary data configuration

  Type_api_camera camera_handle;
  bool camera_started = false;
  bool camera_stopped = true;
  std::thread *camera_capturer_thread;
  std::thread* camera_processing_thread = nullptr; //<! Handle for thread that runs processing loop
  cwipc_pcl_pointcloud current_pcl_pointcloud = nullptr;  //<! Most recent grabbed pointcloud
  ob::Pipeline camera_pipeline;
  // xxxjack k4a_transformation_t transformation_handle = nullptr; //<! k4a structure describing relationship between RGB and D cameras

  moodycamel::BlockingReaderWriterQueue<std::shared_ptr<ob::FrameSet>> captured_frame_queue;
  moodycamel::BlockingReaderWriterQueue<std::shared_ptr<ob::FrameSet>> processing_frame_queue;
  std::shared_ptr<ob::FrameSet> current_captured_frameset;
  bool camera_sync_ismaster;
  bool camera_sync_inuse;
  std::mutex processing_mutex;  //<! Exclusive lock for frame to pointcloud processing.
  std::condition_variable processing_done_cv; //<! Condition variable signalling pointcloud ready
  bool processing_done = false;

  std::string record_to_file;

};
