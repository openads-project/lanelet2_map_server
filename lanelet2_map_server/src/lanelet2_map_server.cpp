#include "lanelet2_map_server/lanelet2_map_server.hpp"

LL2MapServer::LL2MapServer() : Node("ll2_map_server") {
  this->declareParameters();
  this->loadParameters();
  this->setup();
}

void LL2MapServer::declareParameters() {

  random::msg::ParameterDescriptor param_desc;

  param_desc.description = "Path to Lanelet2 map";
  this->declare_parameter("map_filepath", rclcpp::ParameterType::PARAMETER_STRING, param_desc);

  param_desc.description = "Frame ID of Lanelet2 map";
  this->declare_parameter("map_frame_id", map_frame_id_, param_desc);

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

  parameters_callback_ = this->add_on_set_parameters_callback(std::bind(&TestPkg::parametersCallback, this, std::placeholders::_1));

  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
}

rcl_interfaces::msg::SetParametersResult LL2MapServer::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {

  for (const auto& param : parameters) {
    if (param.get_name() == "param") {
      param_ = param.as_double();
      // TODO
    }
  }

  // mark parameter change successful
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

void LL2MapServer::change_params(const std::shared_ptr<lanelet2_map_server_interfaces::srv::ChangeMapParams::Request> request, std::shared_ptr<lanelet2_map_server_interfaces::srv::ChangeMapParams::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received request to change the lanelet2 map parameters!");
  // Perform sanity check of map, before changing parameters and creating service --> e.g. check if map is available etc.
  if(map_sanity_check(request->map_filename, request->map_frame_id, request->origin_lat, request->origin_lon))
  {
    map_filename_ = request->map_filename;
    map_frame_id_ = request->map_frame_id;
    origin_lat_ = request->origin_lat;
    origin_lon_ = request->origin_lon;

    RCLCPP_INFO_STREAM(get_logger(), "Set Lanelet2 Parameters:"
                                    << "\n Map-Filepath: " << map_filename_
                                    << "\n Map-Frame ID: " << map_frame_id_
                                    << "\n Origin Lat: " << origin_lat_
                                    << "\n Origin Lon: " << origin_lon_);

    // Load the map as string
    std::ifstream file(map_filename_);
    map_contents_ = std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    std::vector<rclcpp::Parameter> params{rclcpp::Parameter("map_filepath", map_filename_),
                                          rclcpp::Parameter("map_frame_id", map_frame_id_),
                                          rclcpp::Parameter("map_contents", map_contents_),
                                          rclcpp::Parameter("origin_lat", origin_lat_),
                                          rclcpp::Parameter("origin_lon", origin_lon_)};
    if(!init_)
    {
      // set parameter description
      rcl_interfaces::msg::ParameterDescriptor param_desc;

      param_desc.description = "Path to the Lanelet2 map-file.";
      this->declare_parameter("map_filepath", rclcpp::ParameterType::PARAMETER_STRING, param_desc);
      param_desc.description = "Frame ID of the Lanelet2 map-frame.";
      this->declare_parameter("map_frame_id", rclcpp::ParameterType::PARAMETER_STRING, param_desc);
      param_desc.description = "Contents of the Lanelet2 map-file.";
      this->declare_parameter("map_contents", rclcpp::ParameterType::PARAMETER_STRING, param_desc);
      param_desc.description = "Latitude-Origin of the Lanelet2 map-frame.";
      this->declare_parameter("origin_lat", rclcpp::ParameterType::PARAMETER_DOUBLE, param_desc);
      param_desc.description = "Longitude-Origin of the Lanelet2 map-frame.";
      this->declare_parameter("origin_lon", rclcpp::ParameterType::PARAMETER_DOUBLE, param_desc);
      init_=true;
    }

    this->set_parameters(params);
    this->pub_tf();
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

bool LL2MapServer::map_sanity_check(std::string map_filename, std::string map_frame_id, double origin_lat, double origin_lon)
{
  if(map_filename == map_filename_ && origin_lat == origin_lat_ && origin_lon == origin_lon_ && map_frame_id == map_frame_id_)
  {
    RCLCPP_WARN_STREAM(get_logger(), "Map " << map_filename << " is already loaded with origin (" << origin_lat << " | " << origin_lon << ")");
    return false;
  }
  // Create Projector
  lanelet::projection::UtmProjector proj(lanelet::Origin({origin_lat, origin_lon}));
  // Load Map
  try
  {
    lanelet::LaneletMapPtr map = lanelet::load(map_filename, proj);
    return true;
  }
  catch(const lanelet::IOError& e)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not load map " << map_filename);
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
    RCLCPP_INFO_STREAM(get_logger(), "Broadcasting transform " << t.header.frame_id << " -> " << t.child_frame_id);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LL2MapServer>());
  rclcpp::shutdown();
  return 0;
}
