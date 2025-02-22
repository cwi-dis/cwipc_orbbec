#pragma once

#include <thread>
#include "libobsensor/hpp/Device.hpp"

#include "cwipc_util/api_pcl.h"
#include "cwipc_util/internal.h"

struct OrbbecCameraProcessingParameters {
  bool do_threshold = true;
  double threshold_near = 0.15;         // float, near point for distance threshold
  double threshold_far = 6.0;           // float, far point for distance threshold
  int depth_x_erosion = 0;        // How many valid depth pixels to remove in camera x direction
  int depth_y_erosion = 0;        // How many valid depth pixels to remove in camera y direction
  int32_t color_exposure_time = -1;     // default for manual: 40000;
  int32_t color_whitebalance = -1;   // default for manual: 3160; range(2500-12500)
  int32_t color_backlight_compensation = 0;     // default for manual: 0;
  int32_t color_brightness = 128;        // default for manual: 128;
  int32_t color_contrast = 5;          // default for manual: 5;
  int32_t color_saturation = 32;        // default for manual: 32;
  int32_t color_sharpness = 2;         // default for manual: 2;
  int32_t color_gain = 0;              // default for manual: 100;
  int32_t color_powerline_frequency = 2;     // default for manual: 2;
  bool map_color_to_depth = false; // default DEPTH_TO_COLOR
};

struct OrbbecCameraConfig : CwipcBaseCameraConfig {
  bool disabled = false;  // to easily disable cameras without altering to much the cameraconfig.
  std::shared_ptr<ob::Device> handle = nullptr; // Will be set if camera is opened

  std::string serial;   // Serial number of this camera
  std::string type = "orbbec";       // Camera type (must be realsense)
  std::string filename;   // Filename for offline captures

  pcl::shared_ptr<Eigen::Affine3d> trafo; //!< Transformation matrix from camera coorindates to world coordinates
  pcl::shared_ptr<Eigen::Affine3d> intrinsicTrafo;  //!< offline only: matrix to convert color to depth coordinates

  cwipc_vector cameraposition = { 0, 0, 0 };  //!< Position of this camera in real world coordinates
};

struct OrbbecCaptureConfig : CwipcBaseCaptureConfig {
  int color_height = 720;                     // width of color frame (720, 1080 and various other values allowed, see kinect docs)
  int depth_height = 576;                // width of depth frame (288, 576, 512 and 1024 allowed)
  int fps = 30;                         // capture fps (5, 15 and 30 allowed)
  bool greenscreen_removal = false;   // If true include greenscreen removal
  double height_min = 0.0;        // If height_min != height_max perform height filtering
  double height_max = 0.0;        // If height_min != height_max perform height filtering
  double radius_filter = 0.0;     // If radius_filter > 0 we will remove all points further than radius_filter from the (0,1,0) axis
  int single_tile = -1;       // if singletile >=0 all the points will be the specified integer

  std::string sync_master_serial = "";  // If empty run without sync. If non-empty this camera is the sync master
  bool ignore_sync = false;  // If true dont look at camera master/sub mode in files.
  OrbbecCameraProcessingParameters camera_processing;
  int bt_sensor_orientation = -1; // Override k4abt sensor_orientation (if >= 0)
  int bt_processing_mode = -1;  // Override k4abt processing_mode (if >= 0)
  std::string bt_model_path = "";     // Override k4abt model path
  std::string record_to_directory = ""; // If non-empty all camera streams will be recorded to this directory.
  // We could probably also allow overriding GPU id and model path, but no need for now.
  // per camera data
  std::vector<OrbbecCameraConfig> all_camera_configs;
};

void cwipc_orbbec_log_warning(std::string warning);
extern char** cwipc_orbbec_warning_store;

bool cwipc_orbbec_jsonfile2config(const char* filename, OrbbecCaptureConfig* config, std::string type);
bool cwipc_orbbec_jsonbuffer2config(const char* filename, OrbbecCaptureConfig* config, std::string type);
std::string cwipc_orbbec_config2string(OrbbecCaptureConfig* config);

#ifdef WIN32
  #include <Windows.h>

  inline void cwipc_setThreadName(std::thread* th, const wchar_t* name) {
    HANDLE threadHandle = static_cast<HANDLE>(th->native_handle());
    SetThreadDescription(threadHandle, name);
  }
#else
  inline void cwipc_setThreadName(std::thread* th, const wchar_t* name) {}
#endif
