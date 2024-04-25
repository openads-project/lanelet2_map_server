#include "lanelet2_map_server/lanelet2_map_server.hpp"

LL2MapServer::LL2MapServer() : Node("ll2_map_server") {
  this->declareParameters();
  this->loadParameters();
  this->setup();
}

void LL2MapServer::declareParameters() {

  rcl_interfaces::msg::ParameterDescriptor param_desc;

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
    map_filepath_ = this->get_parameter("map_filepath").as_string();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", "map_filepath");
    exit(EXIT_FAILURE);
  }

  map_frame_id_ = this->get_parameter("map_frame_id").as_string();

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

void LL2MapServer::setup() {

  parameters_callback_ = this->add_on_set_parameters_callback(std::bind(&LL2MapServer::parametersCallback, this, std::placeholders::_1));

  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  this->map_sanity_check(map_filepath_, map_frame_id_, origin_lat_, origin_lon_);
  this->loadMapContents();
  this->pub_tf();
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
  result.successful = this->map_sanity_check(map_filepath, map_frame_id, origin_lat, origin_lon);
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

bool LL2MapServer::map_sanity_check(std::string map_filepath, std::string map_frame_id, double origin_lat, double origin_lon)
{
  if(map_filepath == map_filepath_ && origin_lat == origin_lat_ && origin_lon == origin_lon_ && map_frame_id == map_frame_id_)
  {
    RCLCPP_WARN_STREAM(get_logger(), "Map " << map_filepath << " is already loaded with origin (" << origin_lat << " | " << origin_lon << ")");
    return false;
  }
  // Create Projector
  lanelet::projection::UtmProjector proj(lanelet::Origin({origin_lat, origin_lon}));
  // Load Map
  try
  {
    lanelet::LaneletMapPtr map = lanelet::load(map_filepath, proj);
    return true;
  }
  catch(const lanelet::IOError& e)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not load map " << map_filepath);
    RCLCPP_ERROR_STREAM(get_logger(), e.what());
    return false;
  }
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
