#include "OrbbecConfig.hpp"

#include <fstream>

void OrbbecCaptureConfig::from_json(const json &json_data) {
  CwipcBaseCaptureConfig::_from_json(json_data);
  OrbbecCaptureConfig &config = *this;
  // version and type should already have been checked.
  json system_data = json_data.at("system");
  _CWIPC_CONFIG_JSON_GET(system_data, color_height, config, color_height);
  _CWIPC_CONFIG_JSON_GET(system_data, depth_height, config, depth_height);
  _CWIPC_CONFIG_JSON_GET(system_data, fps, config, fps);
  _CWIPC_CONFIG_JSON_GET(system_data, single_tile, config, single_tile);
  _CWIPC_CONFIG_JSON_GET(system_data, sync_master_serial, config, sync_master_serial);
  _CWIPC_CONFIG_JSON_GET(system_data, ignore_sync, config, ignore_sync);
  _CWIPC_CONFIG_JSON_GET(system_data, record_to_directory, config, record_to_directory);
  _CWIPC_CONFIG_JSON_GET(system_data, color_exposure_time, config.camera_processing, color_exposure_time);
  _CWIPC_CONFIG_JSON_GET(system_data, color_whitebalance, config.camera_processing, color_whitebalance);
  _CWIPC_CONFIG_JSON_GET(system_data, color_brightness, config.camera_processing, color_brightness);
  _CWIPC_CONFIG_JSON_GET(system_data, color_contrast, config.camera_processing, color_contrast);
  _CWIPC_CONFIG_JSON_GET(system_data, color_saturation, config.camera_processing, color_saturation);
  _CWIPC_CONFIG_JSON_GET(system_data, color_gain, config.camera_processing, color_gain);
  _CWIPC_CONFIG_JSON_GET(system_data, color_powerline_frequency, config.camera_processing, color_powerline_frequency);
  _CWIPC_CONFIG_JSON_GET(system_data, map_color_to_depth, config.camera_processing, map_color_to_depth);

  json postprocessing = json_data.at("postprocessing");
  _CWIPC_CONFIG_JSON_GET(postprocessing, greenscreenremoval, config, greenscreen_removal);
  _CWIPC_CONFIG_JSON_GET(postprocessing, height_min, config, height_min);
  _CWIPC_CONFIG_JSON_GET(postprocessing, height_max, config, height_max);
  _CWIPC_CONFIG_JSON_GET(postprocessing, radius_filter, config, radius_filter);

  json depthfilterparameters = postprocessing.at("depthfilterparameters");
  _CWIPC_CONFIG_JSON_GET(depthfilterparameters, do_threshold, config.camera_processing, do_threshold);
  _CWIPC_CONFIG_JSON_GET(depthfilterparameters, threshold_near, config.camera_processing, threshold_near);
  _CWIPC_CONFIG_JSON_GET(depthfilterparameters, threshold_far, config.camera_processing, threshold_far);
  _CWIPC_CONFIG_JSON_GET(depthfilterparameters, depth_x_erosion, config.camera_processing, depth_x_erosion);
  _CWIPC_CONFIG_JSON_GET(depthfilterparameters, depth_y_erosion, config.camera_processing, depth_y_erosion);

  json skeleton = json_data.at("skeleton");
  _CWIPC_CONFIG_JSON_GET(skeleton, sensor_orientation, config, bt_sensor_orientation);
  _CWIPC_CONFIG_JSON_GET(skeleton, processing_mode, config, bt_processing_mode);
  _CWIPC_CONFIG_JSON_GET(skeleton, model_path, config, bt_model_path);

  json cameras = json_data.at("camera");
  int camera_index = 0;
  config.all_camera_configs.clear();

  for (json::iterator it = cameras.begin(); it != cameras.end(); it++) {
      json camera_config_json = *it;
      OrbbecCameraConfig camera_config;
      camera_config._from_json(camera_config_json);
      // xxxjack should check whether the camera with this serial already exists
      config.all_camera_configs.push_back(camera_config);
      camera_index++;
  }
}

void OrbbecCaptureConfig::to_json(json& json_data) {
  CwipcBaseCaptureConfig::_to_json(json_data);
  OrbbecCaptureConfig &config = *this;
    json cameras;
    int camera_index = 0;

    for (OrbbecCameraConfig cd : config.all_camera_configs) {
        json camera;
        _CWIPC_CONFIG_JSON_PUT(camera, serial, cd, serial);
        _CWIPC_CONFIG_JSON_PUT(camera, type, cd, type);
        _CWIPC_CONFIG_JSON_PUT(camera, disabled, cd, disabled);

        if (cd.filename != "") {
            _CWIPC_CONFIG_JSON_PUT(camera, filename, cd, filename);
        }

        camera["trafo"] = {
            {(*cd.trafo)(0, 0), (*cd.trafo)(0, 1), (*cd.trafo)(0, 2), (*cd.trafo)(0, 3)},
            {(*cd.trafo)(1, 0), (*cd.trafo)(1, 1), (*cd.trafo)(1, 2), (*cd.trafo)(1, 3)},
            {(*cd.trafo)(2, 0), (*cd.trafo)(2, 1), (*cd.trafo)(2, 2), (*cd.trafo)(2, 3)},
            {(*cd.trafo)(3, 0), (*cd.trafo)(3, 1), (*cd.trafo)(3, 2), (*cd.trafo)(3, 3)},
        };

        if (cd.intrinsicTrafo != nullptr) {
            camera["intrinsicTrafo"] = {
               {(*cd.intrinsicTrafo)(0, 0), (*cd.intrinsicTrafo)(0, 1), (*cd.intrinsicTrafo)(0, 2), (*cd.intrinsicTrafo)(0, 3)},
               {(*cd.intrinsicTrafo)(1, 0), (*cd.intrinsicTrafo)(1, 1), (*cd.intrinsicTrafo)(1, 2), (*cd.intrinsicTrafo)(1, 3)},
               {(*cd.intrinsicTrafo)(2, 0), (*cd.intrinsicTrafo)(2, 1), (*cd.intrinsicTrafo)(2, 2), (*cd.intrinsicTrafo)(2, 3)},
               {(*cd.intrinsicTrafo)(3, 0), (*cd.intrinsicTrafo)(3, 1), (*cd.intrinsicTrafo)(3, 2), (*cd.intrinsicTrafo)(3, 3)},
            };
        }

        cameras[camera_index] = camera;
        camera_index++;
    }

    json_data["camera"] = cameras;
    json depthfilterparameters;

    _CWIPC_CONFIG_JSON_PUT(depthfilterparameters, do_threshold, config.camera_processing, do_threshold);
    _CWIPC_CONFIG_JSON_PUT(depthfilterparameters, threshold_near, config.camera_processing, threshold_near);
    _CWIPC_CONFIG_JSON_PUT(depthfilterparameters, threshold_far, config.camera_processing, threshold_far);
    _CWIPC_CONFIG_JSON_PUT(depthfilterparameters, depth_x_erosion, config.camera_processing, depth_x_erosion);
    _CWIPC_CONFIG_JSON_PUT(depthfilterparameters, depth_y_erosion, config.camera_processing, depth_y_erosion);

    json postprocessing;
    postprocessing["depthfilterparameters"] = depthfilterparameters;

    _CWIPC_CONFIG_JSON_PUT(postprocessing, greenscreenremoval, config, greenscreen_removal);
    _CWIPC_CONFIG_JSON_PUT(postprocessing, height_min, config, height_min);
    _CWIPC_CONFIG_JSON_PUT(postprocessing, height_max, config, height_max);
    _CWIPC_CONFIG_JSON_PUT(postprocessing, radius_filter, config, radius_filter);

    json_data["postprocessing"] = postprocessing;

    json system_data;
    _CWIPC_CONFIG_JSON_PUT(system_data, color_height, config, color_height);
    _CWIPC_CONFIG_JSON_PUT(system_data, depth_height, config, depth_height);
    _CWIPC_CONFIG_JSON_PUT(system_data, fps, config, fps);
    _CWIPC_CONFIG_JSON_PUT(system_data, single_tile, config, single_tile);
    _CWIPC_CONFIG_JSON_PUT(system_data, sync_master_serial, config, sync_master_serial);
    _CWIPC_CONFIG_JSON_PUT(system_data, ignore_sync, config, ignore_sync);
    _CWIPC_CONFIG_JSON_PUT(system_data, record_to_directory, config, record_to_directory);
    _CWIPC_CONFIG_JSON_PUT(system_data, color_exposure_time, config.camera_processing, color_exposure_time);
    _CWIPC_CONFIG_JSON_PUT(system_data, color_whitebalance, config.camera_processing, color_whitebalance);
    _CWIPC_CONFIG_JSON_PUT(system_data, color_brightness, config.camera_processing, color_brightness);
    _CWIPC_CONFIG_JSON_PUT(system_data, color_contrast, config.camera_processing, color_contrast);
    _CWIPC_CONFIG_JSON_PUT(system_data, color_saturation, config.camera_processing, color_saturation);
    _CWIPC_CONFIG_JSON_PUT(system_data, color_gain, config.camera_processing, color_gain);
    _CWIPC_CONFIG_JSON_PUT(system_data, color_powerline_frequency, config.camera_processing, color_powerline_frequency);
    _CWIPC_CONFIG_JSON_PUT(system_data, map_color_to_depth, config.camera_processing, map_color_to_depth);

    json skeleton;
    _CWIPC_CONFIG_JSON_PUT(skeleton, sensor_orientation, config, bt_sensor_orientation);
    _CWIPC_CONFIG_JSON_PUT(skeleton, processing_mode, config, bt_processing_mode);
    _CWIPC_CONFIG_JSON_PUT(skeleton, model_path, config, bt_model_path);

    json_data["skeleton"] = skeleton;
    json_data["system"] = system_data;
    json_data["version"] = 3;
    json_data["type"] = config.type;
}

bool cwipc_orbbec_jsonfile2config(const char *filename, OrbbecCaptureConfig *config, std::string typeWanted) {
  json json_data;

  try {
    std::ifstream f(filename);

    if (!f.is_open()) {
      _log_error(std::string("CameraConfig ") + filename + " not found");
      return false;
    }

    json_data = json::parse(f);

    int version = 0;
    json_data.at("version").get_to(version);

    if (version != 3) {
      _log_error(std::string("CameraConfig ") + filename + " ignored, is not version 3");
      return false;
    }

    std::string type;
    json_data.at("type").get_to(type);

    if (type != typeWanted) {
      _log_error(std::string("CameraConfig ") + filename + " ignored, is not " + typeWanted + " but " + type);
      return false;
    }

    from_json(json_data, *config);
  } catch (const std::exception &e) {
    _log_error(std::string("CameraConfig ") + filename + ": exception " + e.what());
    return false;
  }

  return true;
}

bool cwipc_orbbec_jsonbuffer2config(const char *jsonBuffer, OrbbecCaptureConfig *config, std::string typeWanted) {
  json json_data;

  try {
    json_data = json::parse(jsonBuffer);

    int version = 0;
    json_data.at("version").get_to(version);

    if (version != 3) {
      _log_error(std::string("CameraConfig ") + "(inline buffer) " + "ignored, is not version 3");
      return false;
    }

    std::string type;
    json_data.at("type").get_to(type);

    if (type != "orbbec") {
      _log_error(std::string("CameraConfig ") + "(inline buffer) " + "ignored, is not orbbec but " + type);
      return false;
    }

    from_json(json_data, *config);
  } catch (const std::exception &e) {
    _log_error(std::string("CameraConfig ") + "(inline buffer) " + ": exception " + e.what());
    return false;
  }

  json dbg_result;
  to_json(dbg_result, *config);

  return true;
}

std::string cwipc_orbbec_config2string(OrbbecCaptureConfig *config) {
  json result;
  to_json(result, *config);

  return result.dump();
}
