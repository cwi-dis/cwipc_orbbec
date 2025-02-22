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

  bool mergedPointcloudIsFresh = false;
  std::condition_variable mergedPointcloudIsFreshCV;

  bool mergedPointcloudWantNew = false;
  std::condition_variable mergedPointcloudWantNewCV;

  std::thread* controlThread = 0;

  void unloadCameras() {
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
      unloadCameras();

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
    unloadCameras();
  }

  virtual bool configReload(const char* filename) = 0;
  virtual bool seek(uint64_t timestamp) = 0;

  virtual bool captureAllCameras() = 0;
  virtual uint64_t getBestTimestamp() = 0;

  virtual OrbbecCameraConfig* getCameraConfig(std::string serial) final {
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
    mergedPointcloudIsFresh = true;
    mergedPointcloudWantNew = false;

    mergedPointcloudIsFreshCV.notify_all();

    mergedPointcloudWantNew = true;
    mergedPointcloudWantNewCV.notify_all();

    if (controlThread && controlThread->joinable()) {
      controlThread->join();
    }

    delete controlThread;
    controlThread = 0;

    for (auto cam : cameras) {
      cam->stop();
    }

    mergedPointcloudIsFresh = false;
    mergedPointcloudWantNew = false;
  }

  virtual cwipc* getPointcloud() final {
    if (cameraCount == 0) {
      return 0;
    }

    // XXX IMPLEMENT ME
    return 0;
  }

  virtual bool pointcouldAvailable(bool wait) final {
    if (cameraCount == 0) {
      return false;
    }

    // XXX IMPLEMENT ME
    return false;
  }
};
