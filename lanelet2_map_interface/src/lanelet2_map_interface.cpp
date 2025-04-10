#include "lanelet2_map_interface/lanelet2_map_interface.hpp"
#include <cctype>

LL2MapInterface::LL2MapInterface(rclcpp::Node& parent_node, std::string map_server_name)
    : parent_node_(parent_node), map_server_name_(map_server_name) {
  // load own parameters
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = "Path to Lanelet2 map";
  parent_node_.declare_parameter("map_filepath", map_filepath_, param_desc);
  map_filepath_ = parent_node_.get_parameter("map_filepath").as_string();

  // Initialize parameter client and event handler
  parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(&parent_node_, map_server_name);
  parameter_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(&parent_node_);

  // Periodically check if the parameter server is available
  startup_timer_ = parent_node_.create_wall_timer(1s, std::bind(&LL2MapInterface::findMapServer, this));
}

void LL2MapInterface::findMapServer() {
  if (!parameter_client_->wait_for_service(0.01s)) {
    if (!rclcpp::ok()) {
      RCLCPP_FATAL(parent_node_.get_logger(),
                   "Interrupted while waiting for the map server ('%s') parameter service, shutting down",
                   map_server_name_.c_str());
      rclcpp::shutdown();
    }
    RCLCPP_WARN(parent_node_.get_logger(), "Waiting for map server ('%s') parameter service ...",
                map_server_name_.c_str());
    return;
  } else {
    auto parameters_future = parameter_client_->get_parameters(
        {"map_frame_id", "map_contents", "origin_lat", "origin_lon"},
        std::bind(&LL2MapInterface::serviceParamsCallback, this, std::placeholders::_1));
    RCLCPP_INFO(parent_node_.get_logger(), "Connected to map server ('%s') parameter service",
                map_server_name_.c_str());

    // Only declare parameters once
    if (!params_declared_) {
      frame_id_callback_handle_ = parameter_sub_->add_parameter_callback(
          "map_frame_id", std::bind(&LL2MapInterface::updateParamsCallback, this, std::placeholders::_1),
          map_server_name_);
      contents_callback_handle_ = parameter_sub_->add_parameter_callback(
          "map_contents", std::bind(&LL2MapInterface::updateParamsCallback, this, std::placeholders::_1),
          map_server_name_);
      origin_lat_callback_handle_ = parameter_sub_->add_parameter_callback(
          "origin_lat", std::bind(&LL2MapInterface::updateParamsCallback, this, std::placeholders::_1),
          map_server_name_);
      origin_lon_callback_handle_ = parameter_sub_->add_parameter_callback(
          "origin_lon", std::bind(&LL2MapInterface::updateParamsCallback, this, std::placeholders::_1),
          map_server_name_);
      params_declared_ = true;
    }
    // workaround to ensure that map is properly loaded on startup (could lead to infinite attempts to load the map, when map-data is actually corrupt)
    if (map_loaded_) startup_timer_->cancel();
  }
}

void LL2MapInterface::updateParamsCallback(const rclcpp::Parameter& p) {
  RCLCPP_INFO_STREAM(parent_node_.get_logger(),
                     "Received an update to parameter " << p.get_name() << "! \n Reloading lanelet2-map!");
  updateMapParam(p);
  bool success = loadMap();
}

void LL2MapInterface::serviceParamsCallback(std::shared_future<std::vector<rclcpp::Parameter>> future) {
  auto result = future.get();
  for (auto& parameter : result) {
    updateMapParam(parameter);
  }
  bool success = loadMap();
}

bool LL2MapInterface::validateParams() {
  if (map_frame_id_.size() == 0) {
    RCLCPP_ERROR_STREAM(parent_node_.get_logger(), "Parameter map_frame_id_ is an empty string!");
    return false;
  }
  if (map_filepath_.size() == 0) {
    RCLCPP_ERROR_STREAM(parent_node_.get_logger(), "Parameter map_filepath_ is an empty string!");
    return false;
  }
  if (map_contents_.size() == 0) {
    RCLCPP_ERROR_STREAM(parent_node_.get_logger(), "Parameter map_contents_ is an empty string!");
    return false;
  }
  if (origin_lat_ == 91.0)  // check if still initialized to invalid value
  {
    RCLCPP_WARN_STREAM(parent_node_.get_logger(), "Parameter origin_lat_ is not set!");
    return false;
  }
  if (origin_lon_ == 181.0)  // check if still initialized to invalid value
  {
    RCLCPP_WARN_STREAM(parent_node_.get_logger(), "Parameter origin_lon_ is not set!");
    return false;
  }
  return true;
}

lanelet::LaneletMapPtr LL2MapInterface::getNonConstMapPtr() {
  if (!map_loaded_) {
    RCLCPP_ERROR(parent_node_.get_logger(), "Lanelet2-Map is currently not loaded. Returning nullptr!");
    return nullptr;
  } else {
    return mapPtr_;
  }
}

lanelet::LaneletMapConstPtr LL2MapInterface::getMapPtr() { return getNonConstMapPtr(); }

std::shared_ptr<lanelet::Projector> LL2MapInterface::getProjectorPtr() {
  if (!map_loaded_) {
    RCLCPP_ERROR(parent_node_.get_logger(), "Lanelet2-Projector is currently not initialized. Returning nullptr!");
    return nullptr;
  } else {
    return utmProjectorPtr_;
  }
}

void LL2MapInterface::updateMapParam(rclcpp::Parameter param) {
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

bool LL2MapInterface::loadMap() {
  // Validate parameters
  if (!validateParams()) {
    return false;
  }

  // Check if location of map file exists, if not create all folders
  std::string map_directory = map_filepath_.substr(0, map_filepath_.find_last_of("/"));
  if (!std::filesystem::exists(map_directory)) {
    RCLCPP_INFO_STREAM(parent_node_.get_logger(), "Creating folder " << map_directory << " for map file.");
    if (!std::filesystem::create_directories(map_directory)) {
      RCLCPP_ERROR_STREAM(parent_node_.get_logger(), "Failed to create folder " << map_directory);
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
    RCLCPP_INFO_STREAM(parent_node_.get_logger(), "Loaded map '" + map_filepath_ + "' successfully!");
    update_pending_ = true;
  } catch (const std::exception& exc) {
    RCLCPP_ERROR_STREAM(parent_node_.get_logger(), "Unable to load " + map_filepath_ + ". Exception: " + exc.what());
    return false;
  }

  return true;
}
