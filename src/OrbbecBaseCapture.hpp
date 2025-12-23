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
        return camera_count; 
    }

    virtual bool is_valid() override final { 
        return camera_count > 0; 
    }
      
    virtual bool config_reload_and_start_capturing(const char* filename) override = 0;

    virtual std::string config_get() override {
        return configuration.to_string();
    }

    virtual bool pointcloud_available(bool wait) override final {
        if (camera_count == 0) {
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
        if (camera_count == 0) {
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
        if (camera_count == 0) {
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
        out2d[0] = u;
        out2d[1] = v;
        return true;
    }

    virtual bool seek(uint64_t timestamp) override = 0;

    bool eof() override {
        return _eof;
    }

public:
  OrbbecCaptureConfig configuration;
protected:
  std::vector<Type_our_camera*> cameras;

  int camera_count = 0;
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

public:

  void _unload_cameras() {
    stop();

    for (auto cam : cameras) {
      delete cam;
    }

    cameras.clear();
    camera_count = 0;
  }

  virtual void stop() final {
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

  virtual OrbbecCameraConfig* get_camera_config(std::string serial) final {
    for (int i = 0; i < configuration.all_camera_configs.size(); i++) {
      if (configuration.all_camera_configs[i].serial == serial) {
        return &configuration.all_camera_configs[i];
      }
    }

    _log_error("Unknown camera " + serial);
    return 0;
  }
protected:

  virtual bool apply_config(const char* configFilename) final {
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
  virtual bool _apply_auto_config() = 0;

  virtual void _init_camera_positions() final {
    for (auto &config : configuration.all_camera_configs) {
      if (config.disabled) {
        continue;
      }

      cwipc_pcl_pointcloud pointCloud(new_cwipc_pcl_pointcloud());
      cwipc_pcl_point point;

      point.x = 0;
      point.y = 0;
      point.z = 0;

      pointCloud->push_back(point);
      transformPointCloud(*pointCloud, *pointCloud, *config.trafo);

      cwipc_pcl_point transformedPoint = pointCloud->points[0];

      config.cameraposition.x = transformedPoint.x;
      config.cameraposition.y = transformedPoint.y;
      config.cameraposition.z = transformedPoint.z;
    }
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

  virtual void _control_thread_main() final {
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
      bool all_captures_ok = _capture_all_cameras();

      if (!all_captures_ok) {
          std::this_thread::yield();
          continue;
      }

      if (stopped) {
          break;
      }

      // And get the best timestamp
      uint64_t timestamp = _get_best_timestamp();
      if (configuration.new_timestamps) {
          timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      }
      // xxxjack current_ts = timestamp;

      // Step 2 - Create pointcloud, and save rgb/depth images if wanted
      cwipc_pcl_pointcloud pcl_pointcloud = new_cwipc_pcl_pointcloud();
      char* error_str = NULL;
      cwipc* newPC = cwipc_from_pcl(pcl_pointcloud, timestamp, &error_str, CWIPC_API_VERSION);

      if (newPC == nullptr) {
          _log_error(std::string("capturer failed to create cwipc pointcloud: ") + (error_str ? error_str : "unknown error"));
          break;
      }

#ifdef xxxjack_notyet
      if (want_auxdata_rgb || want_auxdata_depth) {
          for (auto cam : cameras) {
              cam->save_auxdata_images(newPC, want_auxdata_rgb, want_auxdata_depth);
          }
      }

      if (want_auxdata_skeleton) {
          for (auto cam : cameras) {
              cam->save_auxdata_skeleton(newPC);
          }
      }
#endif
      if (stopped) {
          break;
      }

      // Step 3: start processing frames to pointclouds, for each camera
      for (auto cam : cameras) {
          cam->create_pc_from_frames();
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
          cam->wait_for_pc();
      }

      if (stopped) {
          break;
      }

      // Step 5: merge views
      merge_views();

      if (mergedPC->access_pcl_pointcloud()->size() > 0) {
          _log_debug("Capturer produced merged pointcloud with " + std::to_string(mergedPC->access_pcl_pointcloud()->size()) + " points");
      } else {
          _log_debug("Capturer produced merged pointcloud with ZERO points");

#if 0
          // HACK to make sure the encoder does not get an empty pointcloud
          cwipc_pcl_point point;
          point.x = 1.0;
          point.y = 1.0;
          point.z = 1.0;
          point.rgb = 0.0;
          mergedPC->points.push_back(point);
#endif
      }
      // Signal that a new mergedPC is available. (Note that we acquired the mutex earlier)
      mergedPC_is_fresh = true;
      mergedPC_want_new = false;
      mergedPC_is_fresh_cv.notify_all();
    }
  }

      virtual void merge_views() final {
        cwipc_pcl_pointcloud aligned_cld(mergedPC->access_pcl_pointcloud());
        aligned_cld->clear();

        // Pre-allocate space in the merged pointcloud
        size_t nPoints = 0;
        for (auto cam : cameras) {
            cwipc_pcl_pointcloud cam_cld = cam->get_current_pointcloud();
            if (cam_cld == nullptr) {
                _log_warning("Camera " + cam->serial + " returned NULL cloud, ignoring");
                continue;
            }

            nPoints += cam_cld->size();
        }

        aligned_cld->reserve(nPoints);

        // Now merge all pointclouds
        for (auto cam : cameras) {
            cwipc_pcl_pointcloud cam_cld = cam->get_current_pointcloud();

            if (cam_cld == nullptr) {
                continue;
            }

            *aligned_cld += *cam_cld;
        }

        if (aligned_cld->size() != nPoints) {
            _log_error("Combined pointcloud has different number of points than expected");
        }
    }
public:



  virtual bool _capture_all_cameras() = 0;
  virtual uint64_t _get_best_timestamp() = 0;

};
