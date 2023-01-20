#include "lanelet2_map_server/lanelet2_map_server.hpp"

LL2MapServer::LL2MapServer() : Node("ll2_map_server")
{
  // Initialize service to change the map-parameters
  change_map_srv_ = create_service<lanelet2_map_server_ifs::srv::ChangeMapParams>("~/change_map_parameters", std::bind(&LL2MapServer::change_params, this, std::placeholders::_1, std::placeholders::_2));

}

void LL2MapServer::change_params(const std::shared_ptr<lanelet2_map_server_ifs::srv::ChangeMapParams::Request> request, std::shared_ptr<lanelet2_map_server_ifs::srv::ChangeMapParams::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received request to change the lanelet2 map parameters!");
  // Perform sanity check of map, before changing parameters and creating service --> e.g. check if map is available etc.
  if(map_sanity_check(request->map_filename, request->origin_lat, request->origin_lon))
  {
    map_filename_ = request->map_filename;
    map_frame_id_ = request->map_frame_id;
    origin_lat_ = request->origin_lat;
    origin_lon_ = request->origin_lon;

    if(!init_)
    {
      this->declare_parameter("map_filepath");
      this->declare_parameter("map_frame_id");
      this->declare_parameter("origin_lat");
      this->declare_parameter("origin_lon");
      init_=true;
    }

    RCLCPP_INFO_STREAM(get_logger(), "Set Lanelet2 Parameters:"
                                      << "\n Map-Filepath: " << map_filename_ 
                                      << "\n Map-Frame ID: " << map_frame_id_ 
                                      << "\n Origin Lat: " << origin_lat_
                                      << "\n Origin Lon: " << origin_lon_);

    std::vector<rclcpp::Parameter> params{rclcpp::Parameter("map_filepath", map_filename_),
                                          rclcpp::Parameter("map_frame_id", map_frame_id_),
                                          rclcpp::Parameter("origin_lat", origin_lat_),
                                          rclcpp::Parameter("origin_lon", origin_lon_)};
    this->set_parameters(params);
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

bool LL2MapServer::map_sanity_check(std::string map_filename, double origin_lat, double origin_lon)
{
  if(map_filename == map_filename_ && origin_lat == origin_lat_ && origin_lon == origin_lon_)
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LL2MapServer>());
  rclcpp::shutdown();
  return 0;
}
