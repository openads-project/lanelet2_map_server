#include "lanelet2_map_server/lanelet2_map_server.hpp"
#include <GeographicLib/Geodesic.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <utility>
#include <pugixml.hpp>

LL2MapServer::LL2MapServer() : Node("ll2_map_server") {
  this->declareParameters();
  this->loadParameters();
  this->setup();
}

void LL2MapServer::declareParameters() {

  rcl_interfaces::msg::ParameterDescriptor param_desc;

  param_desc.description = "Automatic map selection";
  this->declare_parameter("use_automatic_map_selection", use_automatic_map_selection_, param_desc);

  param_desc.description = "Manual origin specification";
  this->declare_parameter("use_manual_origin", use_manual_origin_, param_desc);

  param_desc.description = "Directory to search for Lanelet2 map";
  this->declare_parameter("map_directory", rclcpp::ParameterType::PARAMETER_STRING, param_desc);

  param_desc.description = "Path to Lanelet2 map";
  this->declare_parameter("map_filepath", rclcpp::ParameterType::PARAMETER_STRING, param_desc);

  param_desc.description = "Frame ID of Lanelet2 map";
  this->declare_parameter("map_frame_id", map_frame_id_, param_desc);

  param_desc.description = "Raw contents of Lanelet2 map file";
  this->declare_parameter("map_contents", rclcpp::ParameterType::PARAMETER_STRING, param_desc);

  param_desc.description = "Latitude of origin of Lanelet2 map";
  this->declare_parameter("origin_lat", rclcpp::ParameterType::PARAMETER_DOUBLE, param_desc);

  param_desc.description = "Longitude of origin of Lanelet2 map";
  this->declare_parameter("origin_lon", rclcpp::ParameterType::PARAMETER_DOUBLE, param_desc);
}

void LL2MapServer::loadParameters() {

  try {
    use_automatic_map_selection_ = this->get_parameter("use_automatic_map_selection").as_bool();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_INFO(this->get_logger(), "Parameter '%s' is not set. Using default: true", "use_automatic_map_selection");
    use_automatic_map_selection_ = true;
  }

  try {
    use_manual_origin_ = this->get_parameter("use_manual_origin").as_bool();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_INFO(this->get_logger(), "Parameter '%s' is not set. Using default: false", "use_manual_origin");
    use_manual_origin_ = false;
  }

  if(use_automatic_map_selection_) {
    try {
      map_directory_ = this->get_parameter("map_directory").as_string();
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
      RCLCPP_INFO(this->get_logger(), "Parameter '%s' is not set. Using default: %s", "map_directory", map_directory_.c_str());
    }
  }

  if(!use_automatic_map_selection_) {
    try {
      map_filepath_ = this->get_parameter("map_filepath").as_string();
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
        RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", "map_filepath");
        exit(EXIT_FAILURE);
    }
  }

  map_frame_id_ = this->get_parameter("map_frame_id").as_string();

  if(use_manual_origin_) {
    try {
      origin_lat_ = this->get_parameter("origin_lat").as_double();
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
      RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", "origin_lat");
      exit(EXIT_FAILURE);
    }
    try {
      origin_lon_ = this->get_parameter("origin_lon").as_double();
    } catch (rclcpp::exceptions::ParameterUninitializedException&) {
      RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", "origin_lon");
      exit(EXIT_FAILURE);
    }
  }
}

void LL2MapServer::setup() {

  parameters_callback_ = this->add_on_set_parameters_callback(std::bind(&LL2MapServer::parametersCallback, this, std::placeholders::_1));

  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  if(use_automatic_map_selection_) {
    find_available_maps(map_directory_, available_maps_);
    for (auto & map_meta : available_maps_) {
      derive_map_bounds(map_meta);
    }
    RCLCPP_INFO(this->get_logger(), "Discovered %zu Lanelet2 maps in '%s'", available_maps_.size(), map_directory_.c_str());
    if(available_maps_.empty()) {
      RCLCPP_FATAL(this->get_logger(), "No Lanelet2 maps found in '%s'", map_directory_.c_str());
      exit(EXIT_FAILURE);
    }
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

  if(!use_automatic_map_selection_) {
    if(this->map_sanity_check(map_filepath_, origin_lat_, origin_lon_)) {
      this->loadMapContents();
      this->pub_tf();
    }
  } else {
    navsat_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "~/gps/fix", 10, std::bind(&LL2MapServer::navSatFixCallback, this, std::placeholders::_1));
    automatic_map_timer_ = this->create_wall_timer(std::chrono::seconds(1),
      std::bind(&LL2MapServer::automaticMapUpdateTimerCallback, this));
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
}

void LL2MapServer::derive_map_bounds(Lanelet2MapMeta& map_meta) const {
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
    RCLCPP_WARN_STREAM(get_logger(), "No root element found when deriving bounds for '" << map_meta.map_path << "'");
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
    return;
  }

  const Lanelet2MapMeta* selected_map = nullptr;
  for (const auto& map_meta : available_maps_) {
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
    return;
  }

  if (map_filepath_ == selected_map->map_path) {
    return;
  } else {
    map_filepath_ = selected_map->map_path;
    origin_lat_ = selected_map->min_lat;
    origin_lon_ = selected_map->min_lon;
    this->loadMapContents();
    this->pub_tf();
  }
}

rcl_interfaces::msg::SetParametersResult LL2MapServer::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {

  rcl_interfaces::msg::SetParametersResult result;

  bool param_is_updated = false;
  bool map_is_updated = false;
  std::string map_filepath = map_filepath_;
  std::string map_frame_id = map_frame_id_;
  double origin_lat = origin_lat_;
  double origin_lon = origin_lon_;
  for (const auto& param : parameters) {
    if (param.get_name() == "map_filepath") {
      map_filepath = param.as_string();
      param_is_updated = true;
      map_is_updated = true;
    } else if (param.get_name() == "map_frame_id") {
      map_frame_id = param.as_string();
      param_is_updated = true;
    } else if (param.get_name() == "origin_lat") {
      origin_lat = param.as_double();
      param_is_updated = true;
    } else if (param.get_name() == "origin_lon") {
      origin_lon = param.as_double();
      param_is_updated = true;
    }
  }

  // short-circuit if no relevant parameters are updated
  if (!param_is_updated) {
    result.successful = true;
    return result;
  }

  // perform map sanity check
  result.successful = this->map_sanity_check(map_filepath, origin_lat, origin_lon);
  if (!result.successful) {
    result.reason = "Map sanity check failed";
    return result;
  }

  map_filepath_ = map_filepath;
  map_frame_id_ = map_frame_id;
  origin_lat_ = origin_lat;
  origin_lon_ = origin_lon;

  if (map_is_updated) {
    // reload map in timer callback since parameters cannot be updated in this callback
    one_shot_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this]() {
      this->one_shot_timer_->cancel();
      this->loadMapContents();
      this->pub_tf();
    });
  }

  return result;
}

void LL2MapServer::loadMapContents() {

  std::ifstream file(map_filepath_);
  map_contents_ = std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  this->set_parameter(rclcpp::Parameter("map_contents", map_contents_));
  RCLCPP_INFO(this->get_logger(), "Loaded map contents from '%s' to parameter 'map_contents'", map_filepath_.c_str());
}

bool LL2MapServer::map_sanity_check(std::string map_filepath, double origin_lat, double origin_lon) {

  lanelet::projection::UtmProjector proj(lanelet::Origin({origin_lat, origin_lon}));
  try {
    lanelet::LaneletMapPtr map = lanelet::load(map_filepath, proj);
  } catch(const lanelet::IOError& e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load map '" << map_filepath << "': " << e.what());
    return false;
  }

  return true;
}

void LL2MapServer::derive_utm_zone(const double latitude, const double longitude, int& zone, bool& northp)
{
  if(latitude>=0.0) northp = true;
  else northp = false;

  zone = (int)std::floor((longitude + 180.0)/6.0) + 1;
  return;
}

void LL2MapServer::pub_tf()
{
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
    t.header.stamp = this->get_clock()->now();

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
