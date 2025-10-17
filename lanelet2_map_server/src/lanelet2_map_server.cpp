#include "lanelet2_map_server/lanelet2_map_server.hpp"
#include <GeographicLib/Geodesic.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <utility>
#include <pugixml.hpp>

LL2MapServer::LL2MapServer() : Node("ll2_map_server") {

  // General parameters
  this->declareAndLoadParameter("map_frame_id", map_frame_id_, "Frame ID of Lanelet2 map", true, false, false);
  // Automatic map selection parameters
  this->declareAndLoadParameter("use_automatic_map_selection", use_automatic_map_selection_, "Automatic map selection", false, false, true);
  this->declareAndLoadParameter("map_directory", map_directory_, "Directory containing Lanelet2 maps", true, false, false);
  // Map-Server parameters (are reconfigurable and required if automatic map selection is disabled)
  this->declareAndLoadParameter("map_filepath", map_filepath_, "Path to Lanelet2 map", !use_automatic_map_selection_, !use_automatic_map_selection_, false);
  this->declareAndLoadParameter("origin_lat", origin_lat_, "Latitude of origin of Lanelet2 map", !use_automatic_map_selection_, !use_automatic_map_selection_, false);
  this->declareAndLoadParameter("origin_lon", origin_lon_, "Longitude of origin of Lanelet2 map", !use_automatic_map_selection_, !use_automatic_map_selection_, false);
  // Map-Contents is never required and never reconfigurable
  this->declareAndLoadParameter("map_contents", map_contents_, "Contents of Lanelet2 map", false, false, false);

  this->setup();
}

template <typename T>
void LL2MapServer::declareAndLoadParameter(const std::string& name,
                                                         T& param,
                                                         const std::string& description,
                                                         const bool add_to_auto_reconfigurable_params,
                                                         const bool is_required,
                                                         const bool read_only,
                                                         const std::optional<double>& from_value,
                                                         const std::optional<double>& to_value,
                                                         const std::optional<double>& step_value,
                                                         const std::string& additional_constraints) {

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue(param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr(std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value()));
      if (step_value.has_value()) range.set__step(static_cast<T>(step_value.value()));
      param_desc.integer_range = {range};
    } else if constexpr(std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value()));
      if (step_value.has_value()) range.set__step(static_cast<T>(step_value.value()));
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter type of parameter '%s' does not support specifying a range", name.c_str());
    }
  }

  this->declare_parameter(name, type, param_desc);

  try {
    param = this->get_parameter(name).get_value<T>();
    std::stringstream ss;
    ss << "Loaded parameter '" << name << "': ";
    if constexpr(is_vector_v<T>) {
      ss << "[";
      for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
      ss << "]";
    } else {
      ss << param;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Missing required parameter '" << name << "', exiting");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Missing parameter '" << name << "', using default value: ";
      if constexpr(is_vector_v<T>) {
        ss << "[";
        for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
        ss << "]";
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
      this->set_parameters({rclcpp::Parameter(name, rclcpp::ParameterValue(param))});
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) {
      param = p.get_value<T>();
    };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}


rcl_interfaces::msg::SetParametersResult LL2MapServer::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {

  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s' to: %s", param.get_name().c_str(), param.value_to_string().c_str());
        break;
      }
    }
    // handle special cases
    if (param.get_name() == "map_directory" && use_automatic_map_selection_) {
      find_available_maps(map_directory_, available_maps_);
      if(available_maps_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No Lanelet2 maps found in '%s'", map_directory_.c_str());
        unsetMapParameters();
      }
    }
    if ((param.get_name() == "map_filepath" || param.get_name() == "origin_lat" || param.get_name() == "origin_lon") && !use_automatic_map_selection_) {
      if(this->map_sanity_check(map_filepath_, origin_lat_, origin_lon_)) {
        this->updateMapParameters();
        this->pub_tf();
      } else {
        RCLCPP_ERROR(this->get_logger(), "Map sanity check failed for map '%s' with origin (lat=%.9f, lon=%.9f)", map_filepath_.c_str(), origin_lat_, origin_lon_);
        unsetMapParameters();
      }
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}

void LL2MapServer::setup() {

  // callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(std::bind(&LL2MapServer::parametersCallback, this, std::placeholders::_1));
 
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  if(use_automatic_map_selection_) {
    find_available_maps(map_directory_, available_maps_);
    if(available_maps_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No Lanelet2 maps found in '%s'", map_directory_.c_str());
      unsetMapParameters();
    }
    // Create NavSatFix subscription and automatic map update timer
    navsat_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "~/gps/fix", 10, std::bind(&LL2MapServer::navSatFixCallback, this, std::placeholders::_1));
    automatic_map_timer_ = this->create_wall_timer(std::chrono::seconds(1),
      std::bind(&LL2MapServer::automaticMapUpdateTimerCallback, this));
  } else {
    if(this->map_sanity_check(map_filepath_, origin_lat_, origin_lon_)) {
      this->updateMapParameters();
      this->pub_tf();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Map sanity check failed for map '%s' with origin (lat=%.9f, lon=%.9f)", map_filepath_.c_str(), origin_lat_, origin_lon_);
      unsetMapParameters();
    }
  }
}

void LL2MapServer::find_available_maps(const std::string& directory, std::vector<Lanelet2MapMeta>& maps) const {
  maps.clear();
  try {
    for (const auto& entry : std::filesystem::recursive_directory_iterator(directory)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      if (entry.path().extension() == ".osm") {
        Lanelet2MapMeta map_meta;
        map_meta.map_path = entry.path().string();
        maps.emplace_back(map_meta);
      }
    }
  } catch (const std::filesystem::filesystem_error& e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to scan '" << directory << "': " << e.what());
  }
  for (auto & map_meta : maps) {
    derive_map_meta(map_meta);
  }
  RCLCPP_INFO(this->get_logger(), "Discovered %zu Lanelet2 maps in '%s'", maps.size(), directory.c_str());
  for (const auto& map_meta : available_maps_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Map '%s': min_lat=%.9f, min_lon=%.9f, max_lat=%.9f, max_lon=%.9f, diagonal_length=%.3f",
      map_meta.map_path.c_str(),
      map_meta.min_lat,
      map_meta.min_lon,
      map_meta.max_lat,
      map_meta.max_lon,
      map_meta.diagonal_length);
  }
}

void LL2MapServer::derive_map_meta(Lanelet2MapMeta& map_meta) const {
  map_meta.diagonal_length = -1.0;
  pugi::xml_document document;
  const pugi::xml_parse_result result = document.load_file(map_meta.map_path.c_str());
  if (!result) {
    RCLCPP_ERROR_STREAM(get_logger(), "Unable to parse map '" << map_meta.map_path << "': " << result.description());
    return;
  }

  pugi::xml_node root = document.child("osm");
  if (!root) {
    root = document.document_element();
  }
  if (!root) {
    RCLCPP_WARN_STREAM(get_logger(), "No root element found when deriving meta for '" << map_meta.map_path << "'");
    return;
  }

  const double kDoubleMax = std::numeric_limits<double>::max();
  const double kDoubleLowest = std::numeric_limits<double>::lowest();
  double min_lat = kDoubleMax;
  double min_lon = kDoubleMax;
  double max_lat = kDoubleLowest;
  double max_lon = kDoubleLowest;
  bool found_node = false;

  for (pugi::xml_node node : root.children("node")) {
    const double lat = node.attribute("lat").as_double(std::numeric_limits<double>::quiet_NaN());
    const double lon = node.attribute("lon").as_double(std::numeric_limits<double>::quiet_NaN());
    if (std::isnan(lat) || std::isnan(lon)) {
      continue;
    }

    min_lat = std::min(min_lat, lat);
    max_lat = std::max(max_lat, lat);
    min_lon = std::min(min_lon, lon);
    max_lon = std::max(max_lon, lon);
    found_node = true;
  }

  if (!found_node) {
    RCLCPP_WARN_STREAM(get_logger(), "No node elements found while deriving bounds for '" << map_meta.map_path << "'");
    return;
  }

  map_meta.min_lat = min_lat;
  map_meta.max_lat = max_lat;
  map_meta.min_lon = min_lon;
  map_meta.max_lon = max_lon;

  double distance = 0.0;
  GeographicLib::Geodesic::WGS84().Inverse(min_lat, min_lon, max_lat, max_lon, distance);
  map_meta.diagonal_length = distance;
}

void LL2MapServer::navSatFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  current_latitude_ = msg->latitude;
  current_longitude_ = msg->longitude;
  gps_fix_received_ = true;
}

void LL2MapServer::automaticMapUpdateTimerCallback() {
  if (!gps_fix_received_) {
    RCLCPP_WARN(this->get_logger(), "No initial GPS fix received, unable to select map");
    return;
  }

  const Lanelet2MapMeta* selected_map = nullptr;
  for (const auto& map_meta : available_maps_) {
    // check for invalid map
    if (map_meta.diagonal_length <= 0.0) {
      continue;
    }
    // check for maps outside current GPS location
    if (current_latitude_ < map_meta.min_lat || current_latitude_ > map_meta.max_lat ||
        current_longitude_ < map_meta.min_lon || current_longitude_ > map_meta.max_lon) {
      continue;
    }
    if (!selected_map || map_meta.diagonal_length < selected_map->diagonal_length) {
      if(map_sanity_check(map_meta.map_path, map_meta.min_lat, map_meta.min_lon)) {
        RCLCPP_DEBUG(this->get_logger(), "Map sanity check passed for '%s'", map_meta.map_path.c_str());
        selected_map = &map_meta;
      } else {
        RCLCPP_WARN(this->get_logger(), "Map sanity check failed for '%s'", map_meta.map_path.c_str());
        continue;
      }
    }
  }

  if (!selected_map) {
    RCLCPP_WARN(this->get_logger(), "No suitable map found for current GPS location (lat=%.9f, lon=%.9f)", current_latitude_, current_longitude_);
    unsetMapParameters();
    return;
  }

  if (map_filepath_ == selected_map->map_path) {
    RCLCPP_DEBUG(this->get_logger(), "Currently loaded map '%s' is still valid", map_filepath_.c_str());
    return;
  } else {
    map_filepath_ = selected_map->map_path;
    origin_lat_ = selected_map->min_lat;
    origin_lon_ = selected_map->min_lon;
    this->updateMapParameters();
    this->pub_tf();
  }
}

void LL2MapServer::updateMapParameters() {
  std::ifstream file(map_filepath_);
  map_contents_ = std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  this->set_parameter(rclcpp::Parameter("map_filepath", map_filepath_));
  this->set_parameter(rclcpp::Parameter("map_contents", map_contents_));
  this->set_parameter(rclcpp::Parameter("origin_lat", origin_lat_));
  this->set_parameter(rclcpp::Parameter("origin_lon", origin_lon_));
  RCLCPP_INFO(this->get_logger(), "Loaded map contents from '%s' to parameter 'map_contents'", map_filepath_.c_str());
}

void LL2MapServer::unsetMapParameters() {
  this->set_parameter(rclcpp::Parameter("map_filepath", ""));
  this->set_parameter(rclcpp::Parameter("map_contents", ""));
  this->set_parameter(rclcpp::Parameter("origin_lat", 0.0));
  this->set_parameter(rclcpp::Parameter("origin_lon", 0.0));
  RCLCPP_INFO(this->get_logger(), "Unset map parameters");
}

bool LL2MapServer::map_sanity_check(std::string map_filepath, double origin_lat, double origin_lon) const {

  lanelet::projection::UtmProjector proj(lanelet::Origin({origin_lat, origin_lon}));
  try {
    lanelet::LaneletMapPtr map = lanelet::load(map_filepath, proj);
  } catch(const lanelet::IOError& e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load map '" << map_filepath << "': " << e.what());
    return false;
  }

  return true;
}

void LL2MapServer::derive_utm_zone(const double latitude, const double longitude, int& zone, bool& northp) const {
  if(latitude>=0.0) northp = true;
  else northp = false;

  zone = (int)std::floor((longitude + 180.0)/6.0) + 1;
  return;
}

void LL2MapServer::pub_tf() const {
  geometry_msgs::msg::TransformStamped t;

  // Create Projector without offset
  lanelet::projection::UtmProjector proj_utm(lanelet::Origin({origin_lat_, origin_lon_}), false);
  lanelet::BasicPoint3d origin_utm = proj_utm.forward(lanelet::GPSPoint({origin_lat_,origin_lon_,0.0}));

  t.transform.translation.x = origin_utm.x();
  t.transform.translation.y = origin_utm.y();
  t.transform.translation.z = origin_utm.z();

  t.transform.rotation.x = 0;
  t.transform.rotation.y = 0;
  t.transform.rotation.z = 0;
  t.transform.rotation.w = 1;

  bool northp;
  int zone;
  derive_utm_zone(origin_lat_, origin_lon_, zone, northp);

  std::string hemisphere;
  if(northp) hemisphere="N";
  else hemisphere="S";

  t.header.frame_id ="utm_"+std::to_string(zone)+hemisphere;
  t.child_frame_id = map_frame_id_;
  t.header.stamp = now();

  // Send the transformation
  tf_static_broadcaster_->sendTransform(t);
  RCLCPP_INFO(get_logger(), "Broadcast static transform from '%s' to '%s'", t.header.frame_id.c_str(), t.child_frame_id.c_str());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LL2MapServer>());
  rclcpp::shutdown();
  return 0;
}
