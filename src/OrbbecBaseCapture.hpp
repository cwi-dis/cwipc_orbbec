#pragma once

#include <string>
#include <thread>
#include <condition_variable>
#include <pcl/common/transforms.h>
#include <cwipc_util/internal.h>

#include "OrbbecConfig.hpp"

template<class CameraType> class OrbbecBaseCapture : public CwipcBaseCapture {
protected:
  std::string CLASSNAME;
  std::vector<CameraType*> cameras;

  bool stopped = false;

  uint64_t startTime = 0;
  int numberOfPointCloudsProduced = 0;

  cwipc* mergedPC;
  std::mutex mergedPC_mutex;

  bool mergedPC_is_fresh = false;
  std::condition_variable mergedPC_is_fresh_cv;

  bool mergedPC_want_new = false;
  std::condition_variable mergedPC_want_new_cv;

  std::thread* controlThread = 0;

  void _unload_cameras() {
    stop();

    for (auto cam : cameras) {
      delete cam;
    }

    cameras.clear();
    cameraCount = 0;
  }

  virtual bool _apply_default_config() = 0;

  virtual bool apply_config(const char* configFilename) final {
    if (configFilename == 0 || *configFilename == '\0') {
      configFilename = "cameraconfig.json";
    }

    if (strcmp(configFilename, "auto") == 0) {
      return _apply_default_config();
    }

    if (configFilename[0] == '{') {
      return cwipc_orbbec_jsonbuffer2config(configFilename, &configuration, type);
    }

    const char* extension = strrchr(configFilename, '.');
    if (extension != 0 && strcmp(extension, ".json") == 0) {
      return cwipc_orbbec_jsonfile2config(configFilename, &configuration, type);
    }

    return false;
  }

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
    bool errorOccurred = false;

    for (auto cam : cameras) {
      if (cam->isSyncMaster()) {
        continue;
      }

      if (!cam->start()) {
        errorOccurred = true;
      }
    }

    for (auto cam : cameras) {
      if (!cam->isSyncMaster()) {
        continue;
      }

      if (!cam->start()) {
        errorOccurred = true;
      }
    }

    if (errorOccurred) {
      cwipc_orbbec_log_warning("Not all cameras could be started");
      _unload_cameras();

      return;
    }

    startTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    for (auto cam : cameras) {
      if (cam->isSyncMaster()) {
        continue;
      }

      cam->start_capturer();
    }

    for (auto cam : cameras) {
      if (!cam->isSyncMaster()) {
        continue;
      }

      cam->start_capturer();
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
          if (cam->eof) {
              eof = true;
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
          //std::cerr << CLASSNAME << ": xxxjack not all captures succeeded. Retrying." << std::endl;
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
#ifdef CWIPC_DEBUG
          std::cerr << CLASSNAME << ": cwipc_from_pcl returned error: " << error_str << std::endl;
#endif
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
#ifdef CWIPC_DEBUG
          std::cerr << CLASSNAME << ": capturer produced a merged cloud of " << mergedPC->count() << " points" << std::endl;
#endif
      } else {
#ifdef CWIPC_DEBUG
          std::cerr << CLASSNAME << ": Warning: capturer got an empty pointcloud\n";
#endif

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
                cwipc_orbbec_log_warning("Camera " + cam->serial + " returned NULL cloud, ignoring");
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
            cwipc_orbbec_log_warning("Combined pointcloud has different number of points than expected");
        }
    }
public:
  OrbbecCaptureConfig configuration;
  int cameraCount = 0;
  bool eof = false;

  OrbbecBaseCapture(const std::string& classname) : CLASSNAME(classname) {
  }

  virtual ~OrbbecBaseCapture() {
    uint64_t stopTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    _unload_cameras();
  }

  virtual bool config_reload(const char* filename) = 0;
  virtual bool seek(uint64_t timestamp) = 0;

  virtual bool _capture_all_cameras() = 0;
  virtual uint64_t _get_best_timestamp() = 0;

  virtual OrbbecCameraConfig* get_camera_config(std::string serial) final {
    for (int i = 0; i < configuration.all_camera_configs.size(); i++) {
      if (configuration.all_camera_configs[i].serial == serial) {
        return &configuration.all_camera_configs[i];
      }
    }

    cwipc_orbbec_log_warning("Unknown camera " + serial);
    return 0;
  }

  virtual void stop() final {
    stopped = true;
    mergedPC_is_fresh = true;
    mergedPC_want_new = false;

    mergedPC_is_fresh_cv.notify_all();

    mergedPC_want_new = true;
    mergedPC_want_new_cv.notify_all();

    if (controlThread && controlThread->joinable()) {
      controlThread->join();
    }

    delete controlThread;
    controlThread = 0;

    for (auto cam : cameras) {
      cam->stop();
    }

    mergedPC_is_fresh = false;
    mergedPC_want_new = false;
  }

  virtual cwipc* get_pointcloud() final {
    if (cameraCount == 0) {
      return 0;
    }

    // XXX IMPLEMENT ME
    return 0;
  }

  virtual bool pointcloud_available(bool wait) final {
    if (cameraCount == 0) {
      return false;
    }

    // XXX IMPLEMENT ME
    return false;
  }
};
