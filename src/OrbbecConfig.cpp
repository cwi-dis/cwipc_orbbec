#include "OrbbecConfig.hpp"

#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#define JSON_GET(jsonobj, name, config, attr) if (jsonobj.contains(#name)) jsonobj.at(#name).get_to(config.attr)
#define JSON_PUT(jsonobj, name, config, attr) jsonobj[#name] = config.attr

static std::string orbbec_most_recent_warning;
char** cwipc_orbbec_warning_store;

void cwipc_orbbec_log_warning(std::string warning) {
  std::cerr << "cwipc_orbbec: Warning:" << warning << std::endl;

  if (cwipc_orbbec_warning_store) {
    orbbec_most_recent_warning = warning;
    *cwipc_orbbec_warning_store = (char*)orbbec_most_recent_warning.c_str();
  }
}

void from_json(const json &json_data, OrbbecCaptureConfig &config) {
  json system_data = json_data.at("system");
  JSON_GET(system_data, color_height, config, color_height);
  JSON_GET(system_data, depth_height, config, depth_height);
  JSON_GET(system_data, fps, config, fps);
  JSON_GET(system_data, single_tile, config, single_tile);
  JSON_GET(system_data, sync_master_serial, config, sync_master_serial);
  JSON_GET(system_data, ignore_sync, config, ignore_sync);
  JSON_GET(system_data, record_to_directory, config, record_to_directory);
  JSON_GET(system_data, color_exposure_time, config.camera_processing, color_exposure_time);
  JSON_GET(system_data, color_whitebalance, config.camera_processing, color_whitebalance);
  JSON_GET(system_data, color_brightness, config.camera_processing, color_brightness);
  JSON_GET(system_data, color_contrast, config.camera_processing, color_contrast);
  JSON_GET(system_data, color_saturation, config.camera_processing, color_saturation);
  JSON_GET(system_data, color_gain, config.camera_processing, color_gain);
  JSON_GET(system_data, color_powerline_frequency, config.camera_processing, color_powerline_frequency);
  JSON_GET(system_data, map_color_to_depth, config.camera_processing, map_color_to_depth);

  json postprocessing = json_data.at("postprocessing");
  JSON_GET(postprocessing, greenscreenremoval, config, greenscreen_removal);
  JSON_GET(postprocessing, height_min, config, height_min);
  JSON_GET(postprocessing, height_max, config, height_max);
  JSON_GET(postprocessing, radius_filter, config, radius_filter);

  json depthfilterparameters = postprocessing.at("depthfilterparameters");
  JSON_GET(depthfilterparameters, do_threshold, config.camera_processing, do_threshold);
  JSON_GET(depthfilterparameters, threshold_near, config.camera_processing, threshold_near);
  JSON_GET(depthfilterparameters, threshold_far, config.camera_processing, threshold_far);
  JSON_GET(depthfilterparameters, depth_x_erosion, config.camera_processing, depth_x_erosion);
  JSON_GET(depthfilterparameters, depth_y_erosion, config.camera_processing, depth_y_erosion);

  json skeleton = json_data.at("skeleton");
  JSON_GET(skeleton, sensor_orientation, config, bt_sensor_orientation);
  JSON_GET(skeleton, processing_mode, config, bt_processing_mode);
  JSON_GET(skeleton, model_path, config, bt_model_path);

  json cameras = json_data.at("camera");
  int camera_index = 0;
  config.all_camera_configs.clear();

  for (json::iterator it = cameras.begin(); it != cameras.end(); it++)
  {
    json camera = *it;
    OrbbecCameraConfig cd;
    pcl::shared_ptr<Eigen::Affine3d> default_trafo(new Eigen::Affine3d());
    default_trafo->setIdentity();
    cd.trafo = default_trafo;

    JSON_GET(camera, serial, cd, serial);
    JSON_GET(camera, type, cd, type);
    JSON_GET(camera, disabled, cd, disabled);
    JSON_GET(camera, filename, cd, filename);

    if (camera.contains("trafo")) {
      for (int x = 0; x < 4; x++) {
        for (int y = 0; y < 4; y++) {
          (*cd.trafo)(x, y) = camera["trafo"][x][y];
        }
      }
    }

    if (camera.contains("intrinsicTrafo")) {
      if (cd.intrinsicTrafo == nullptr) {
        pcl::shared_ptr<Eigen::Affine3d> intrinsic_trafo(new Eigen::Affine3d());
        cd.intrinsicTrafo = intrinsic_trafo;
      }

      for (int x = 0; x < 4; x++) {
        for (int y = 0; y < 4; y++) {
          (*cd.intrinsicTrafo)(x, y) = camera["intrinsicTrafo"][x][y];
        }
      }
    }

    config.all_camera_configs.push_back(cd);
    camera_index++;
  }
}

void to_json(json& json_data, const OrbbecCaptureConfig& config) {
    json cameras;
    int camera_index = 0;

    for (OrbbecCameraConfig cd : config.all_camera_configs) {
        json camera;
        JSON_PUT(camera, serial, cd, serial);
        JSON_PUT(camera, type, cd, type);
        JSON_PUT(camera, disabled, cd, disabled);

        if (cd.filename != "") {
            JSON_PUT(camera, filename, cd, filename);
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

    JSON_PUT(depthfilterparameters, do_threshold, config.camera_processing, do_threshold);
    JSON_PUT(depthfilterparameters, threshold_near, config.camera_processing, threshold_near);
    JSON_PUT(depthfilterparameters, threshold_far, config.camera_processing, threshold_far);
    JSON_PUT(depthfilterparameters, depth_x_erosion, config.camera_processing, depth_x_erosion);
    JSON_PUT(depthfilterparameters, depth_y_erosion, config.camera_processing, depth_y_erosion);

    json postprocessing;
    postprocessing["depthfilterparameters"] = depthfilterparameters;

    JSON_PUT(postprocessing, greenscreenremoval, config, greenscreen_removal);
    JSON_PUT(postprocessing, height_min, config, height_min);
    JSON_PUT(postprocessing, height_max, config, height_max);
    JSON_PUT(postprocessing, radius_filter, config, radius_filter);

    json_data["postprocessing"] = postprocessing;

    json system_data;
    JSON_PUT(system_data, color_height, config, color_height);
    JSON_PUT(system_data, depth_height, config, depth_height);
    JSON_PUT(system_data, fps, config, fps);
    JSON_PUT(system_data, single_tile, config, single_tile);
    JSON_PUT(system_data, sync_master_serial, config, sync_master_serial);
    JSON_PUT(system_data, ignore_sync, config, ignore_sync);
    JSON_PUT(system_data, record_to_directory, config, record_to_directory);
    JSON_PUT(system_data, color_exposure_time, config.camera_processing, color_exposure_time);
    JSON_PUT(system_data, color_whitebalance, config.camera_processing, color_whitebalance);
    JSON_PUT(system_data, color_brightness, config.camera_processing, color_brightness);
    JSON_PUT(system_data, color_contrast, config.camera_processing, color_contrast);
    JSON_PUT(system_data, color_saturation, config.camera_processing, color_saturation);
    JSON_PUT(system_data, color_gain, config.camera_processing, color_gain);
    JSON_PUT(system_data, color_powerline_frequency, config.camera_processing, color_powerline_frequency);
    JSON_PUT(system_data, map_color_to_depth, config.camera_processing, map_color_to_depth);

    json skeleton;
    JSON_PUT(skeleton, sensor_orientation, config, bt_sensor_orientation);
    JSON_PUT(skeleton, processing_mode, config, bt_processing_mode);
    JSON_PUT(skeleton, model_path, config, bt_model_path);

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
      cwipc_orbbec_log_warning(std::string("CameraConfig ") + filename + " not found");
      return false;
    }

    json_data = json::parse(f);

    int version = 0;
    json_data.at("version").get_to(version);

    if (version != 3) {
      cwipc_orbbec_log_warning(std::string("CameraConfig ") + filename + " ignored, is not version 3");
      return false;
    }

    std::string type;
    json_data.at("type").get_to(type);

    if (type != typeWanted) {
      cwipc_orbbec_log_warning(std::string("CameraConfig ") + filename + " ignored, is not " + typeWanted + " but " + type);
      return false;
    }

    from_json(json_data, *config);
  } catch (const std::exception &e) {
    cwipc_orbbec_log_warning(std::string("CameraConfig ") + filename + ": exception " + e.what());
    return false;
  }

  return true;
}

bool cwipc_k4a_jsonbuffer2config(const char *jsonBuffer, OrbbecCaptureConfig *config, std::string typeWanted) {
  json json_data;

  try {
    json_data = json::parse(jsonBuffer);

    int version = 0;
    json_data.at("version").get_to(version);

    if (version != 3) {
      cwipc_orbbec_log_warning(std::string("CameraConfig ") + "(inline buffer) " + "ignored, is not version 3");
      return false;
    }

    std::string type;
    json_data.at("type").get_to(type);

    if (type != "orbbec") {
      cwipc_orbbec_log_warning(std::string("CameraConfig ") + "(inline buffer) " + "ignored, is not orbbec but " + type);
      return false;
    }

    from_json(json_data, *config);
  } catch (const std::exception &e) {
    cwipc_orbbec_log_warning(std::string("CameraConfig ") + "(inline buffer) " + ": exception " + e.what());
    return false;
  }

  json dbg_result;
  to_json(dbg_result, *config);

  return true;
}

std::string cwipc_k4a_config2string(OrbbecCaptureConfig *config) {
  json result;
  to_json(result, *config);

  return result.dump();
}
