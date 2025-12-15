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

  cwipc* mergedPointcloud;
  std::mutex mergedPointcloudMutex;

  bool mergedPC_is_fresh = false;
  std::condition_variable mergedPC_is_fresh_cv;

  bool mergedPC_want_new = false;
  std::condition_variable mergedPointcloudWantNewCV;

  std::thread* controlThread = 0;

  void _unload_cameras() {
    stop();

    for (auto cam : cameras) {
      delete cam;
    }

    cameras.clear();
    cameraCount = 0;
  }

  virtual bool applyDefaultConfig() = 0;

  virtual bool applyConfig(const char* configFilename) final {
    if (configFilename == 0 || *configFilename == '\0') {
      configFilename = "cameraconfig.json";
    }

    if (strcmp(configFilename, "auto") == 0) {
      return applyDefaultConfig();
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

  virtual void initCameraPositions() final {
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

  virtual void startCameras() final {
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

      cam->startCapturer();
    }

    for (auto cam : cameras) {
      if (!cam->isSyncMaster()) {
        continue;
      }

      cam->startCapturer();
    }
  }

  virtual void controlThreadFunction() final {
    // XXX IMPLEMENT ME
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
    mergedPointcloudWantNewCV.notify_all();

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
