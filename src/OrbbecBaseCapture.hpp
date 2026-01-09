#pragma once

#include <string>
#include <thread>
#include <condition_variable>
#include <pcl/common/transforms.h>

#include "cwipc_util/capturers.hpp"
#include "OrbbecConfig.hpp"

template<class Type_api_camera, class Type_our_camera> class OrbbecBaseCapture : public CwipcBaseCapture {
public:
    using CwipcBaseCapture::CwipcBaseCapture;


  virtual ~OrbbecBaseCapture() {
      uint64_t stopTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      _unload_cameras();
  }

    virtual int get_camera_count() override final { 
        return cameras.size(); 
    }

    virtual bool is_valid() override final { 
        return cameras.size() > 0; 
    }

    virtual bool config_reload_and_start_capturing(const char* configFilename) override final{
        _unload_cameras();

        //
        // Read the configuration.
        //
        if (!_apply_config(configFilename)) {
            return false;
        }

        auto camera_config_count = configuration.all_camera_configs.size();
        if (camera_config_count == 0) {
            return false;
        }

        //
        // Initialize hardware capture setting (for all cameras)
        //
        if (!_init_hardware_for_all_cameras()) {
            // xxxjack we should really close all cameras too...
            camera_config_count = 0;
            return false;
        }

        _setup_inter_camera_sync();

        // Now we have all the configuration information. Create our K4ACamera objects.
        if (!_create_cameras()) {
            _unload_cameras();
            return false;
        }

        if (!_check_cameras_connected()) {
            _unload_cameras();
            return false;
        }
        _start_cameras();

        //
        // start our run thread (which will drive the capturers and merge the pointclouds)
        //

        stopped = false;
        control_thread = new std::thread(&OrbbecBaseCapture::_control_thread_main, this);
        _cwipc_setThreadName(control_thread, L"cwipc_orbbec::control_thread");

        return true;
    }

    virtual std::string config_get() override {
        return configuration.to_string();
    }

    /// Tell the capturer that each point cloud should also include RGB and/or D images and/or RGB/D capture timestamps.
    virtual void request_auxiliary_data(bool rgb, bool depth, bool timestamps, bool skeleton) override final {
#ifdef xxxjack_notyet
      configuration.auxData.want_auxdata_rgb = rgb;
      configuration.auxData.want_auxdata_depth = depth;
      configuration.auxData.want_auxdata_timestamps = timestamps;
#endif
    }

    virtual bool pointcloud_available(bool wait) override final {
        if (!is_valid() == 0) {
            return false;
        }
#ifdef xxxjack_implement_me
        _request_new_pointcloud();
        std::this_thread::yield();
        std::unique_lock<std::mutex> mylock(mergedPC_mutex);
        auto duration = std::chrono::seconds(wait ? 1 : 0);

        mergedPC_is_fresh_cv.wait_for(mylock, duration, [this] {
            return mergedPC_is_fresh;
        });

        return mergedPC_is_fresh;
#else
        // XXX IMPLEMENT ME
        return false;
#endif
    }

    virtual cwipc* get_pointcloud() override final {
        if (!is_valid()) {
          _log_warning("get_pointcloud: returning NULL, no cameras");
          return nullptr;
        }
#ifdef xxxjack_implement_me
        _request_new_pointcloud();
        // Wait for a fresh mergedPC to become available.
        // Note we keep the return value while holding the lock, so we can start the next grab/process/merge cycle before returning.
        cwipc* rv;

        {
            std::unique_lock<std::mutex> mylock(mergedPC_mutex);
            mergedPC_is_fresh_cv.wait(mylock, [this] {
                return mergedPC_is_fresh;
            });

            assert(mergedPC_is_fresh);
            mergedPC_is_fresh = false;
            numberOfPCsProduced++;
            rv = mergedPC;
            mergedPC = nullptr;

            if (rv == nullptr) {
              _log_warning("get_pointcloud: returning NULL, even though mergedPC_is_fresh");
            }
        }

        _request_new_pointcloud();
        return rv;
#else
        // XXX IMPLEMENT ME
        return nullptr;
#endif
    }

    virtual float get_pointSize() override final {
        if (!is_valid()) {
            return 0;
        }

        float rv = 99999;

        for (auto cam : cameras) {
            if (cam->pointSize < rv) {
                rv = cam->pointSize;
            }
        }

        if (rv > 9999) {
            rv = 0;
        }

        return rv;
    }

    virtual bool map2d3d(int tile, int x_2d, int y_2d, int d_2d, float* out3d) override final
    {
#ifdef xxxjack_notyet
        for (auto cam : cameras) {
            if (tile == (1 << cam->camera_index)) {
                return cam->map2d3d(x_2d, y_2d, d_2d, out3d);
            }
        }
#endif
        return false;
    }
    
    virtual bool mapcolordepth(int tile, int u, int v, int* out2d) override final
    {
        // For Kinect the RGB and D images have the same coordinate system.
        // xxxjack check this is true for Orbbec too.
        out2d[0] = u;
        out2d[1] = v;
        return true;
    }

    bool eof() override {
        return _eof;
    }

    virtual bool seek(uint64_t timestamp) override = 0;

protected:
  /// Load configuration from file or string.
  virtual bool _apply_config(const char* configFilename) override final {
    // xxxjack keep configuration.auxData
    if (configFilename == 0 || *configFilename == '\0') {
      configFilename = "cameraconfig.json";
    }

    if (strcmp(configFilename, "auto") == 0) {
      return _apply_auto_config();
    }

    if (configFilename[0] == '{') {
      return configuration.from_string(configFilename, type);
    }

    const char* extension = strrchr(configFilename, '.');
    if (extension != 0 && strcmp(extension, ".json") == 0) {
      return configuration.from_file(configFilename, type);
    }

    return false;
  }
  /// Load default configuration based on hardware cameras connected.
  virtual bool _apply_auto_config() override = 0;
  /// Get configuration for a single camera, by serial number.
  OrbbecCameraConfig* get_camera_config(std::string serial) {
    for (int i = 0; i < configuration.all_camera_configs.size(); i++) {
      if (configuration.all_camera_configs[i].serial == serial) {
        return &configuration.all_camera_configs[i];
      }
    }

    _log_error("Unknown camera " + serial);
    return 0;
  }
  /// Create our wrapper around a single camera. Here because it needs to be templated.
  virtual inline Type_our_camera *_create_single_camera(Type_api_camera _handle, OrbbecCaptureConfig& configuration, int _camera_index, OrbbecCameraConfig& _camData) final {
      return new Type_our_camera(_handle, configuration, _camera_index, _camData);
  }

  /// Setup camera synchronization (if needed).
  virtual bool _setup_inter_camera_sync() override final {
      // Nothing to do for K4A: real cameras need some setup, but it is done
      // in K4ACamera::_prepare_config_for_starting_camera().
      return true;
  }
  /// xxxjack another one?
  virtual void _initial_camera_synchronization() override {
  }



  virtual void _start_cameras() final {
    bool start_error = false;
    for (auto cam: cameras) {
        if (!cam->pre_start_all_cameras()) {
            start_error = true;
        }
    }
    // xxxjack Check for Orbbec. K4A wants master camera started _last_, because then
    // any recordings will be automatically starting at the same frame. Need to check
    // that this is also true for Orbbec.
    for (auto cam : cameras) {
      if (cam->is_sync_master()) {
        continue;
      }

      if (!cam->start_camera()) {
        start_error = true;
      }
    }

    for (auto cam : cameras) {
      if (!cam->is_sync_master()) {
        continue;
      }

      if (!cam->start_camera()) {
        start_error = true;
      }
    }
    for (auto cam : cameras) {
      cam->post_start_all_cameras();
    }

    if (start_error) {
      _log_error("Not all cameras could be started");
      _unload_cameras();

      return;
    }

    starttime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    for (auto cam : cameras) {
      if (cam->is_sync_master()) {
        continue;
      }

      cam->start_camera_streaming();
    }

    for (auto cam : cameras) {
      if (!cam->is_sync_master()) {
        continue;
      }

      cam->start_camera_streaming();
    }
  }

  virtual void _unload_cameras() override final {
    _stop_cameras();

    for (auto cam : cameras) {
      delete cam;
    }

    cameras.clear();
    _log_debug("deleted all cameras");
  }

  virtual void _stop_cameras() override final {
    stopped = true;
    mergedPC_is_fresh = true;
    mergedPC_want_new = false;

    mergedPC_is_fresh_cv.notify_all();

    mergedPC_want_new = true;
    mergedPC_want_new_cv.notify_all();

    if (control_thread && control_thread->joinable()) {
      control_thread->join();
    }

    delete control_thread;
    control_thread = 0;

    for (auto cam : cameras) {
      cam->stop_camera();
    }

    mergedPC_is_fresh = false;
    mergedPC_want_new = false;
  }

protected:

  void _control_thread_main() {
    _log_debug_thread("processing thread started");
    _initial_camera_synchronization();
    // XXX IMPLEMENT ME
    while (!stopped) {
      {
        std::unique_lock<std::mutex> mylock(mergedPC_mutex);
        mergedPC_want_new_cv.wait(mylock, [this] { return mergedPC_want_new; });
      }
      //check EOF:
      for (auto cam : cameras) {
          if (cam->_eof) {
              _eof = true;
              stopped = true;
              break;
          }
      }

      if (stopped) {
          break;
      }

      assert(cameras.size() > 0);
      // Step one: grab frames from all cameras. This should happen as close together in time as possible,
      // because that gives use he biggest chance we have the same frame (or at most off-by-one) for each
      // camera.
      uint64_t timestamp = 0;
      bool all_captures_ok = _capture_all_cameras(timestamp);

      if (!all_captures_ok) {
          std::this_thread::yield();
          continue;
      }

      if (stopped) {
          break;
      }

      if (configuration.new_timestamps) {
          timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      }

      // Step 2 - Create pointcloud, and save rgb/depth images if wanted
      if (configuration.debug) _log_debug("creating pc with ts=" + std::to_string(timestamp));
      cwipc_pcl_pointcloud pcl_pointcloud = new_cwipc_pcl_pointcloud();
      cwipc* newPC = cwipc_from_pcl(pcl_pointcloud, timestamp, nullptr, CWIPC_API_VERSION);


#ifdef xxxjack_notyet
      for (auto cam : cameras) {
          cam->save_frameset_auxdata(newPC);
      }

#endif
      if (stopped) {
          break;
      }

      // Step 3: start processing frames to pointclouds, for each camera
      for (auto cam : cameras) {
          cam->process_pointcloud_from_frameset();
      }

      if (stopped) {
          break;
      }

      // Lock mergedPC already while we are waiting for the per-camera
      // processing threads. This so the main thread doesn't go off and do
      // useless things if it is calling available(true).
      std::unique_lock<std::mutex> mylock(mergedPC_mutex);
      if (mergedPC && mergedPC_is_fresh) {
          mergedPC->free();
          mergedPC = nullptr;
      }

      if (stopped) {
          break;
      }

      mergedPC = newPC;

      // Step 4: wait for frame processing to complete.
      for (auto cam : cameras) {
          cam->wait_for_pointcloud_processed();
      }

      if (stopped) {
          break;
      }

      // Step 5: merge views
      _merge_camera_pointclouds();

      if (mergedPC->access_pcl_pointcloud()->size() > 0) {
          _log_debug("merged pointcloud has  " + std::to_string(mergedPC->access_pcl_pointcloud()->size()) + " points");
      } else {
          _log_warning("merged pointcloud is empty");
      }
      // Signal that a new mergedPC is available. (Note that we acquired the mutex earlier)
      mergedPC_is_fresh = true;
      mergedPC_want_new = false;
      mergedPC_is_fresh_cv.notify_all();
    }
    if (configuration.debug) _log_debug_thread("processing thread exiting");
  }

  // xxxjack needs to go
  virtual uint64_t _get_best_timestamp() = 0;
  virtual bool _capture_all_cameras(uint64_t& timestamp) = 0;


  void _request_new_pointcloud() {
      std::unique_lock<std::mutex> mylock(mergedPC_mutex);

      if (!mergedPC_want_new && !mergedPC_is_fresh) {
          mergedPC_want_new = true;
          mergedPC_want_new_cv.notify_all();
      }
  }

  void _merge_camera_pointclouds() {
      cwipc_pcl_pointcloud aligned_cld(mergedPC->access_pcl_pointcloud());
      aligned_cld->clear();
      // Pre-allocate space in the merged pointcloud
      size_t nPoints = 0;

      for (auto cam : cameras) {
          cwipc_pcl_pointcloud cam_cld = cam->access_current_pcl_pointcloud();

          if (cam_cld == 0) {
              _log_warning("_merge_camera_pointclouds: camera pointcloud is null for some camera" );
              continue;
          }
          nPoints += cam_cld->size();
      }

      aligned_cld->reserve(nPoints);

      // Now merge all pointclouds
      for (auto cam : cameras) {
          cwipc_pcl_pointcloud cam_cld = cam->access_current_pcl_pointcloud();

          if (cam_cld == NULL) {
              continue;
          }

          *aligned_cld += *cam_cld;
      }
      if (aligned_cld->size() != nPoints) {
          _log_error("Combined pointcloud has different number of points than expected");
      }

      // No need to merge aux_data: already inserted into mergedPC by each camera
  }    
public:
  OrbbecCaptureConfig configuration;
protected:
  std::vector<Type_our_camera*> cameras;

  bool stopped = false;
  bool _eof = false;

  uint64_t starttime = 0;
  int numberOfPCsProduced = 0;

  cwipc* mergedPC;
  std::mutex mergedPC_mutex;

  bool mergedPC_is_fresh = false;
  std::condition_variable mergedPC_is_fresh_cv;

  bool mergedPC_want_new = false;
  std::condition_variable mergedPC_want_new_cv;

  std::thread* control_thread = 0;
};
