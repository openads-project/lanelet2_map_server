#include "lanelet2_map_server/lanelet2_map_server.hpp"

LL2MapServer::LL2MapServer() : Node("ll2_map_server")
{
  // Initialize service to change the map-parameters
  change_map_srv_ = this->create_service<lanelet2_map_manager_srvs::srv::ChangeMapParams>("~/change_map_parameters", std::bind(&LL2MapServer::change_params, this, std::placeholders::_1, std::placeholders::_2));
  
  // Initialize publisher to indicate a change of the map to different nodes
  map_change_pub_ = this->create_publisher<lanelet2_map_manager_msgs::msg::MapChange>("~/map_changed", 1);

}

void LL2MapServer::change_params(const std::shared_ptr<lanelet2_map_manager_srvs::srv::ChangeMapParams::Request> request, std::shared_ptr<lanelet2_map_manager_srvs::srv::ChangeMapParams::Response> response)
{
  // Perform sanity check of map, before changing parameters and creating service --> e.g. check if map is available etc.
  if(map_sanity_check(request->map_filename, request->origin_lat, request->origin_lon))
  {
    map_filename_ = request->map_filename;
    map_frame_id_ = request->map_frame_id;
    origin_lat_ = request->origin_lat;
    origin_lon_ = request->origin_lon;
    RCLCPP_INFO_STREAM(this->get_logger(), "Set lanelet2-map to " << map_filename_ << "\n Origin Lat: " << origin_lat_ << "\n Origin Lon: " << origin_lon_);
    if(!params_set_)
    {
      params_set_ = true;
      provide_map_srv_ = this->create_service<lanelet2_map_manager_srvs::srv::ProvideMapParams>("~/provide_map_parameters", std::bind(&LL2MapServer::provide_params, this, std::placeholders::_1, std::placeholders::_2));
      RCLCPP_INFO(this->get_logger(), "Setting up service to provide the lanelet2 parameters!");
    }
    else
    {
      // Publish message to notify all nodes that the map has changed
      auto msg = lanelet2_map_manager_msgs::msg::MapChange();
      msg.stamp = this->now();
      msg.map_changed = true;
      map_change_pub_->publish(msg);
    }
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
    RCLCPP_WARN_STREAM(this->get_logger(), "Map " << map_filename << " is already loaded with origin (" << origin_lat << " | " << origin_lon << ")");
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
    RCLCPP_ERROR_STREAM(this->get_logger(), "Could not load map " << map_filename);
    RCLCPP_ERROR_STREAM(this->get_logger(), e.what());
    return false;
  }
}

void LL2MapServer::provide_params(const std::shared_ptr<lanelet2_map_manager_srvs::srv::ProvideMapParams::Request> request, std::shared_ptr<lanelet2_map_manager_srvs::srv::ProvideMapParams::Response> response)
{
  response->map_filename = map_filename_;
  response->map_frame_id = map_frame_id_;
  response->origin_lat = origin_lat_;
  response->origin_lon = origin_lon_;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LL2MapServer>());
  rclcpp::shutdown();
  return 0;
}
