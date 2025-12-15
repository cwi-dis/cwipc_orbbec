#pragma once

#include <string>
#include <cwipc_util/internal.h>

#include "libobsensor/hpp/Frame.hpp"
#include "readerwriterqueue.h"

#include "OrbbecConfig.hpp"

template<typename ApiCameraType> class OrbbecBaseCamera : public CwipcBaseCamera {
protected:
  std::string CLASSNAME;
  ApiCameraType* camera;

  OrbbecCaptureConfig& config;
  OrbbecCameraConfig& cameraConfig;

  bool stopped = true;
  bool cameraStarted = false;
  bool processingDone = false;

  bool cameraIsMaster;
  bool cameraIsUsed;
  bool heightFilteringEnabled;

  std::string recordToFile;

  moodycamel::BlockingReaderWriterQueue<ob::Frame> capturedFrameQueue;
  moodycamel::BlockingReaderWriterQueue<ob::Frame> processingFrameQueue;

  virtual void _start_capture_thread() = 0;
  virtual void _capture_thread_main() = 0;

public:
  std::string serial;
  bool eof = false;
  int cameraIndex;

  OrbbecBaseCamera(const std::string& classname, ApiCameraType* camera, OrbbecCaptureConfig& config, int cameraIndex, OrbbecCameraConfig& cameraConfig) :
    CLASSNAME(classname),
    camera(camera),
    config(config),
    cameraIndex(cameraIndex),
    cameraConfig(cameraConfig),
    serial(cameraConfig.serial),
    cameraIsMaster(cameraConfig.serial == config.sync_master_serial),
    cameraIsUsed(config.sync_master_serial != ""),
    heightFilteringEnabled(config.height_min != config.height_max),
    capturedFrameQueue(1),
    processingFrameQueue(1) {
  }

  virtual ~OrbbecBaseCamera() {
  }

  virtual bool start() = 0;
  virtual void start_capturer() = 0;
  virtual void stop() = 0;

  virtual bool isSyncMaster() {
    return cameraIsMaster;
  }

  virtual uint64_t getCaptureTimestamp() final {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  }
};
