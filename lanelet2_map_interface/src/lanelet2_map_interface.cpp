// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <utility>

#include "lanelet2_map_interface/lanelet2_map_interface.hpp"

/**
 * @brief Resolves a relative string filepath
 *
 * Interprets paths relative to $ROS_HOME, $HOME/.ros, or current working directory.
 * If the path is absolute, it is returned as-is.
 *
 * @param[in] path_string (relative) path
 * @return resolved path
 */
std::filesystem::path resolveFilepath(const std::string& path_string) {
  std::filesystem::path path(path_string);
  if (path_string.empty()) return path;
  if (!path.has_root_path()) {
    const char* ros_home_ptr = std::getenv("ROS_HOME");
    if (ros_home_ptr != nullptr && !std::string(ros_home_ptr).empty()) {
      path = std::filesystem::path(std::string(ros_home_ptr));
    } else {
      const char* home_dir = std::getenv("HOME");
      if (home_dir != nullptr && !std::string(home_dir).empty()) {
        path = std::filesystem::path(std::string(home_dir) + "/.ros");
      } else {
        path = std::filesystem::current_path();
      }
    }
    path.append(path_string);
  }
  path = path.lexically_normal();
  return path;
}

Lanelet2MapInterface::Lanelet2MapInterface(rclcpp::Node& parent_node, std::string map_server_name)
    : parent_node_(&parent_node),
      map_filepath_(std::string(".lanelet2_map_interface/") + std::string(parent_node.get_fully_qualified_name()) + "/map.osm"),
      map_server_name_(map_server_name) {
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = "Local filepath to where map from map server is written to (relative to $ROS_HOME)";
  try {
    parent_node_->declare_parameter("map_filepath", map_filepath_, param_desc);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException& ex) {
    RCLCPP_WARN(parent_node_->get_logger(), "Parameter 'map_filepath' already declared: %s", ex.what());
  }
  map_filepath_ = parent_node_->get_parameter("map_filepath").as_string();
  map_filepath_ = resolveFilepath(map_filepath_).string();
  RCLCPP_INFO(parent_node_->get_logger(), "Loaded parameter 'map_filepath': %s", map_filepath_.c_str());

  // Initialize parameter client and event handler
  parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(parent_node_, map_server_name);
  parameter_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(parent_node_);

  // Periodically check if the parameter server is available
  startup_timer_ =
      parent_node_->create_wall_timer(std::chrono::seconds(1), std::bind(&Lanelet2MapInterface::findMapServer, this));
}

void Lanelet2MapInterface::findMapServer() {
  if (!parameter_client_->wait_for_service(std::chrono::milliseconds(10))) {
    RCLCPP_WARN(parent_node_->get_logger(), "Waiting for map server ('%s') parameter service ...", map_server_name_.c_str());
    return;
  } else {
    startup_timer_->cancel();
    auto parameters_future =
        parameter_client_->get_parameters({"map_frame_id", "map_contents", "origin_lat", "origin_lon"},
                                          std::bind(&Lanelet2MapInterface::serviceParamsCallback, this, std::placeholders::_1));
    RCLCPP_INFO(parent_node_->get_logger(), "Connected to map server ('%s') parameter service", map_server_name_.c_str());

    // Only declare parameters once
    if (!params_declared_) {
      frame_id_callback_handle_ = parameter_sub_->add_parameter_callback(
          "map_frame_id", std::bind(&Lanelet2MapInterface::updateParamsCallback, this, std::placeholders::_1), map_server_name_);
      contents_callback_handle_ = parameter_sub_->add_parameter_callback(
          "map_contents", std::bind(&Lanelet2MapInterface::updateParamsCallback, this, std::placeholders::_1), map_server_name_);
      origin_lat_callback_handle_ = parameter_sub_->add_parameter_callback(
          "origin_lat", std::bind(&Lanelet2MapInterface::updateParamsCallback, this, std::placeholders::_1), map_server_name_);
      origin_lon_callback_handle_ = parameter_sub_->add_parameter_callback(
          "origin_lon", std::bind(&Lanelet2MapInterface::updateParamsCallback, this, std::placeholders::_1), map_server_name_);
      params_declared_ = true;
    }
  }
}

void Lanelet2MapInterface::updateParamsCallback(const rclcpp::Parameter& p) {
  RCLCPP_INFO_STREAM(parent_node_->get_logger(), "Parameter '" << p.get_name() << "' changed, reloading map");
  updateMapParam(p);
  std::ignore = loadMap();
}

void Lanelet2MapInterface::serviceParamsCallback(std::shared_future<std::vector<rclcpp::Parameter>> future) {
  auto result = future.get();
  for (auto& parameter : result) {
    updateMapParam(parameter);
  }
  std::ignore = loadMap();
}

bool Lanelet2MapInterface::validateParams() {
  if (map_frame_id_.size() == 0) {
    RCLCPP_ERROR_STREAM(parent_node_->get_logger(), "Parameter 'map_frame_id_' is empty");
    return false;
  }
  if (map_filepath_.size() == 0) {
    RCLCPP_ERROR_STREAM(parent_node_->get_logger(), "Parameter 'map_filepath_' is empty");
    return false;
  }
  if (map_contents_.size() == 0) {
    RCLCPP_ERROR_STREAM(parent_node_->get_logger(), "Parameter 'map_contents_' is empty");
    return false;
  }
  if (origin_lat_ == std::numeric_limits<double>::quiet_NaN()) {
    RCLCPP_WARN_STREAM(parent_node_->get_logger(), "Parameter 'origin_lat_' is not set");
    return false;
  }
  if (origin_lon_ == std::numeric_limits<double>::quiet_NaN()) {
    RCLCPP_WARN_STREAM(parent_node_->get_logger(), "Parameter 'origin_lon_' is not set");
    return false;
  }
  return true;
}

lanelet::LaneletMapPtr Lanelet2MapInterface::getNonConstMapPtr() {
  if (!map_loaded_) {
    return nullptr;
  } else {
    return mapPtr_;
  }
}

lanelet::LaneletMapConstPtr Lanelet2MapInterface::getMapPtr() { return getNonConstMapPtr(); }

std::shared_ptr<lanelet::Projector> Lanelet2MapInterface::getProjectorPtr() {
  if (!map_loaded_) {
    return nullptr;
  } else {
    return utmProjectorPtr_;
  }
}

void Lanelet2MapInterface::updateMapParam(rclcpp::Parameter param) {
  if (param.get_name() == "map_frame_id") {
    map_frame_id_ = param.value_to_string();
  }
  if (param.get_name() == "map_contents") {
    map_contents_ = param.value_to_string();
  }
  if (param.get_name() == "origin_lat") {
    origin_lat_ = param.as_double();
  }
  if (param.get_name() == "origin_lon") {
    origin_lon_ = param.as_double();
  }
}

bool Lanelet2MapInterface::loadMap() {
  // Validate parameters
  if (!validateParams()) {
    return false;
  }

  // Check if location of map file exists, if not create all folders
  std::string map_directory = map_filepath_.substr(0, map_filepath_.find_last_of("/"));
  if (!std::filesystem::exists(map_directory)) {
    if (!std::filesystem::create_directories(map_directory)) {
      RCLCPP_ERROR_STREAM(parent_node_->get_logger(), "Failed to create directory '" << map_directory << "'");
      return false;
    }
  }

  // Save map_contents to file
  std::ofstream map_file;
  map_file.open(map_filepath_);
  map_file << map_contents_;
  map_file.close();

  try {
    // Load map from file
    utmProjectorPtr_ = std::make_shared<lanelet::projection::UtmProjector>(lanelet::Origin({origin_lat_, origin_lon_}));
    mapPtr_ = lanelet::load(map_filepath_, *utmProjectorPtr_);
    map_loaded_ = true;
    RCLCPP_INFO_STREAM(parent_node_->get_logger(), "Loaded map '" + map_filepath_ + "'");
    update_pending_ = true;
  } catch (const std::exception& exc) {
    RCLCPP_ERROR_STREAM(parent_node_->get_logger(), "Failed to load map '" + map_filepath_ + "': " + exc.what());
    return false;
  }

  return true;
}
